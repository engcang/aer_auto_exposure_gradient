#include "aer_auto_exposure_gradient/auto_exp.h"



namespace exp_node 
{
	cv::Mat lookUpTable_01 (1, 256, CV_8U);
	cv::Mat lookUpTable_05 (1, 256, CV_8U);
	cv::Mat lookUpTable_08 (1, 256, CV_8U);
	cv::Mat lookUpTable_1(1, 256, CV_8U);
	cv::Mat lookUpTable_12(1, 256, CV_8U);
	cv::Mat lookUpTable_15(1, 256, CV_8U);
	cv::Mat lookUpTable_19(1, 256, CV_8U);
	cv::Mat lookUpTable_metric(1, 256, CV_8U);

	ExpNode::ExpNode (const ros::NodeHandle & nh, const ros::NodeHandle & pnh) : nh_(nh)
    {
    	std::cout <<"the image topic given in launch file? :"<< nh.getParam("/image_topic", image_topic)<<"\n";
        std::cout <<"the value of image topic is : "<< image_topic<<"\n";
        std::cout <<"the kp given in launch file? :"<< nh.getParam("/kp", kp)<<"\n";
        std::cout <<"the value of kp is : "<< kp<<"\n";
        std::cout <<"the upper_shutter given in launch file? :"<< nh.getParam("/upper_shutter", upper_shutter)<<"\n";
        std::cout <<"the value of upper_shutter is : "<< upper_shutter<<"\n";
        std::cout <<"the lower_shutter given in launch file? :"<< nh.getParam("/lower_shutter", lower_shutter)<<"\n";
        std::cout <<"the value of lower_shutter is : "<< lower_shutter<<"\n";
        std::cout <<"the gain_max given in launch file? :"<< nh.getParam("/gain_max", gain_max)<<"\n";
        std::cout <<"the value of gain_max is : "<< gain_max<<"\n";
        
    	generate_LUT();
    	img_sub_ = nh_.subscribe<sensor_msgs::CompressedImage>(image_topic, 1, &ExpNode::CameraCb, this);
    	gain_sub_ = nh_.subscribe<std_msgs::Float32>("/telicam/gain", 1, &ExpNode::GainCb, this);
		exp_sub_ = nh_.subscribe<std_msgs::Float32>("/telicam/exposure", 1, &ExpNode::ExposureCb, this);
		gain_pub_ = nh_.advertise<std_msgs::Float32>("/telicam/set_gain", 3);
		exp_pub_ = nh_.advertise<std_msgs::Float32>("/telicam/set_exposure", 3);
    }
	


	void ExpNode::GainCb (const std_msgs::Float32::ConstPtr &msg)
	{
		gain_cur_data_ = msg->data;
	}
	void ExpNode::ExposureCb (const std_msgs::Float32::ConstPtr &msg)
	{
		exp_cur_data_ = msg->data;
	}
	void ExpNode::CameraCb (const sensor_msgs::CompressedImage::ConstPtr &msg)
	{ 
		if (check_rate)
		{
			try
			{
				high_resolution_clock::time_point t1 = high_resolution_clock::now();

				check_rate = false;
				cv::Mat image_current;
				cv::Mat image_capture;

				//cv::Size size(512,612); // may want to try size(408,342) if speed is limited
				// cv::Size size(342,408);

				// image_capture = cv_bridge::toCvCopy(msg, "mono8")->image;
				image_capture = cv_bridge::toCvCopy(msg, "rgb8")->image;

				// cv::cvtColor(image_capture, image_capture, cv::COLOR_BGR2GRAY);
				cv::cvtColor(image_capture, image_current, cv::COLOR_BGR2GRAY);
				// cv::resize(image_capture, image_current, size);
				high_resolution_clock::time_point t2 = high_resolution_clock::now();

				///////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Call the image processing funciton here (i.e. the gamma processing), returning a float point gamma value
				///////////////////////////////////////////////////////////////////////////////////////////////////////////
				
				// loop to call image_gradient_gamma function to obtain image gradient of each gamma
				// manually adjust the possible gamma values and the number of gamma to use
				for (int i=0; i<7; ++i)
				{
				   metric[i]= image_gradient_gamma(image_current, i)/1000000.0; // passing the corresponding index
				   // std::cout << "\nmetric " << i << " is:" <<metric[i] <<std::endl;
				   // std::cout << "  " <<metric[i] <<std::endl; // comment
				}
                                
				// loop to find out the index that correslated to the optimum/maximum gamma value
				double temp = -1.0;				
				for(int i = 0; i < 7; i++){
					if (metric[i] > temp){
						temp = metric[i];
						gamma_index = i;
					} // end of if
				} // end of for loop 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////  Curve Fitting  ///////////////////////////////////////////////


				// Call the curve fitting function to find out coefficient
				double * coeff_curve;
				coeff_curve = curveFit( gamma, metric);				
	
				double coeff[6];
				for ( int i = 0; i < 6; i++ ) {
      					
					coeff[i] = *(coeff_curve+i);
					// std::cout << "\ncoeff " << i << " is:" << coeff[i] << std::endl; // comment
   				}

				double metric_check = 0.0;

				max_gamma = findRoots1(coeff, metric[gamma_index]); // calling function findRoots1 to find opt_gamma
				
				// std::cout << "\nopt_gamma now is:  " << max_gamma << std::endl; //comment
				
				for (int i=0; i<6; i++) 
				{
					metric_check = metric_check + coeff[i] * pow(max_gamma,5-i);
				}
								

				// std::cout << "metric_check = " << metric_check << std::endl; // comment

				if (max_gamma < 1.0/1.9 || max_gamma > 1.9)	
				{
                    // find out the optimum gamma value associated with highest image gradient
					max_gamma = gamma[gamma_index];
				}
				else if (metric[gamma_index] > metric_check)
				{
					max_gamma = gamma[gamma_index];
				}
				


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                               
				// alpha value refers to Shim's 2014 paper
				if (max_gamma < 1)
					// alpha = 0.5;
					alpha = 1.0; //original paper used 0.5
                if (max_gamma >= 1)
					alpha = 0.5;
				

    			// get the current shutter and gain
				shutter_cur = exp_cur_data_ / 1000000.0; // unit from micro-second to second
    			gain_cur = gain_cur_data_;
				
				expCur = log2( (f_number*f_number)/( shutter_cur* pow(2,(gain_cur/6.0)) ) ); // The 7.84 here is because the camera we are using has a F-number of 2.8
				
				// This update function was implemented in Shim's 2018 paper which is an update version of his 2014 paper
				R = d * tan( (2 - max_gamma) * atan2(1,d) - atan2(1,d) ) + 1;
				expNew = (1 + alpha * kp * (R-1)) * expCur;
 	
				// Shim's 2014 version of update function
				//expNew = (1+ kp * alpha * (1-max_gamma)) * expCur; 
			
				shutter_new = ( f_number*f_number) * 1000000.0/(pow(2, expNew)); //[unit: micro-second]  Note: 7.84 = 2.8*2.8
				// std::cout << "\ngain: " << round(gain_cur) << "   shutter_new: "<< shutter_new << std::endl; //


				if (shutter_new > upper_shutter)
        		{
            		gain_flag= true;
            		shutter_new=upper_shutter;
        		}
                else if (shutter_new<lower_shutter)
        		{
            		gain_flag= true;
            		shutter_new=lower_shutter;
                }
                else {gain_flag=false;}
                


                if (gain_flag==true)
            	{
            		gain_new = 6.0*(expCur-expNew)+gain_cur;
			
                	if (gain_new > gain_max)
                		gain_new = gain_max;
                	else if (gain_new < -0.0)
                		gain_new = -0.0;
					gain_flag = false;                    		
            	}
                else if  (shutter_new < upper_shutter && shutter_new > lower_shutter)
            	{
                	gain_new = 0.0;
					//gain_flag = false;
                	//gain_cur=gain_new;
            	}

				///////////////////////////// Comment or delete the following three lines in implementation ////////////////
				// std::cout << "\ngain: " << round(gain_cur) << "   shutter_new: "<< shutter_new << std::endl;
				// std::cout << "\ncurrent opt met: " << metric[gamma_index] << "; met at 1.0: " << metric[3]<<std::endl;

				// std::cout << "max gamma: " << max_gamma << " ; gain_new: " << gain_new << "; shutter_cur: "<< shutter_cur*1000000.0 << " ; shutter_new:  " << shutter_new << std::endl;

				///////////////////////////////////////////////////////////////////
				// Then call the function to determine what exposure time and gain to update
				// may input the gain and exposure time update
				std_msgs::Float32 gain_msg_, exp_msg_;
				gain_msg_.data = gain_new;
				exp_msg_.data = shutter_new;
				gain_pub_.publish(gain_msg_);
				exp_pub_.publish(exp_msg_);    			
				high_resolution_clock::time_point t3 = high_resolution_clock::now();

				ROS_WARN("CV %.1fms, CAL %.1fms, max gamma %.1f, gain_pre %.1f, gain_new %.1f, exp_pre %.1f, exp_new %.1f", 
					duration_cast<microseconds>(t2-t1).count()/1e3, duration_cast<microseconds>(t3-t2).count()/1e3,
					max_gamma, gain_cur, gain_new, shutter_cur, shutter_new);
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("Could not convert img");
			}

		}
		else // keeping the if-else statement here is because this makes easier to add delay
		{
			
			check_rate = true;
		}

	}

	double ExpNode::image_gradient_gamma(cv::Mat &src_img, int j)
	{

	// Accepting the raw image and the index of gamma value as input argument

	    cv::Mat res = src_img.clone();
	    ///////////////////// The following computes the image gradient of the gamma-processed image /////////////////////
	    cv::Mat grad_x, grad_y;
	    cv::Mat abs_grad_x, abs_grad_y, dst_img;

	    cv::Mat weight_ori = cv::Mat::ones(res.rows,res.cols,CV_64FC1);
	    
		// Using the corresponding index to find out the correct lookuptable to use
			if (j == 0){
				cv::LUT(src_img, lookUpTable_01, res);
			}
			else if (j == 1){
				cv::LUT(src_img, lookUpTable_05, res);
			}
			else if (j == 2){
				cv::LUT(src_img, lookUpTable_08, res);
			}
			else if (j == 3){
				cv::LUT(src_img, lookUpTable_1, res);
			}
			else if (j == 4){
				cv::LUT(src_img, lookUpTable_12, res);
			}
			else if (j == 5){
				cv::LUT(src_img, lookUpTable_15, res);
			}
			else if (j == 6){
				cv::LUT(src_img, lookUpTable_19, res);
			}

	    // Define variables that will be used in the sobel gradient determination function
	    int scale = 1;
	    int delta = 0;
	    int ddepth = CV_8UC1;
	
	    // Call the Sobel function to determine gradient image in x and y direction
	
	    // Gradient X
	    cv::Sobel(res, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
	    cv::convertScaleAbs(grad_x, abs_grad_x);

	    // Gradient Y
	    cv::Sobel(res, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
	    cv::convertScaleAbs(grad_y, abs_grad_y);

	    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst_img);

	    ////////////////////// The following computes the gradient metric based on the gradient image ///////////////////////////
    
	    // Method 1: simple sumation of total gradient
	    //double metric= cv::sum(dst_img)[0]; // simple sum of total gradient as metric (Alternative 1 of the Shim's metric) 
	    
	    // Method 2: Shim's 2014 gradient metric function
	    // Using the metric equation given in Shim's 2014 paper
	    cv::LUT(dst_img, lookUpTable_metric, res);
	    res.convertTo(res, CV_64FC1);
    
	    double metric= cv::sum(res)[0];

	    return metric;
	} 


	void ExpNode::generate_LUT ()
	{
		// Need to keep the same gamma values as in imageCallBack
		double gamma[7]={1.0/1.9, 1.0/1.5, 1.0/1.2 ,1.0, 1.2, 1.5, 1.9}; 		
		
		double sigma = 255.0 * met_act_thresh; // The sigma value is used in Shim's 2014 paper as a activation threshhold, the paper used a value of 0.06    
    	//double lamda = 1000.0; // The lamda value used in Shim's 2014 paper as a control parameter to adjust the mapping tendency (larger->steeper) 


		uchar* p;
		uchar* q = lookUpTable_metric.ptr();

		for (int j = 0; j < 7; j++){
			if (j == 0){
				p = lookUpTable_01.ptr();
			}
			else if (j == 1){
				p = lookUpTable_05.ptr();
			}
			else if (j == 2){
				p = lookUpTable_08.ptr();
			}
			else if (j == 3){
				p = lookUpTable_1.ptr();
			}
			else if (j == 4){
				p = lookUpTable_12.ptr();
			}
			else if (j == 5){
				p = lookUpTable_15.ptr();
			}
			else if (j == 6){
				p = lookUpTable_19.ptr();
			}
			
    			for( int i = 0; i < 256; ++i)
    			{  p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 1/gamma[j]) * 255.0);
       
       			// The following if statement to create a lookup table based on the activation threshold value in Shim's 2014 paper
				if (i >= sigma){
					q[i] = 255 * ( ( log10( lamda * ((i-sigma)/255.0) + 1) ) / ( log10( lamda * ((255.0-sigma)/255.0) + 1) ) );
	    			}
				else{
					q[i] = 0;
					}
				} // end of for loop with index i
		}// end of for loop with index j
	} // end of generate_LUT()
	

	double  ExpNode::findRoots1 (double a[6], double check)
	{
		static double roots1[4];
		double ad[5];
		double opt_gamma = 997.0, met_temp;
		ad[4] = 5 * a[0];
		ad[3] = 4 * a[1];
		ad[2] = 3 * a[2];
		ad[1] = 2 * a[3];
		ad[0] = a[4];
		Eigen::MatrixXd companion_mat (4, 4);

		for (int n = 0; n < 4; n++)
		{
			for (int m = 0; m < 4; m++)
				{
				 if (n == m + 1)
				 	companion_mat (n, m) = 1.0;
				 if (m == 4 - 1)
					companion_mat (n, m) = -ad[n] / ad[4];
			} // end of for loop with index m
		} // end of for loop with index n

		Eigen::MatrixXcd eig = companion_mat.eigenvalues ();
		for (int i = 0; i < 4; i++)
		{
		 	met_temp = 0.0; // met_temp is used to check if the root can return a larger metric      
		 	if (std::imag (eig (i)) == 0) // if statement to determine whether or not root is true
		  	{
		  		roots1[i] = std::real(eig (i));	 
		  	}
		  	else
		  	{

		  		roots1[i] = 1000; // if root is imaginary, assign an overshoot value
		  	}
		  if ((roots1[i] < 2.0) && (roots1[i] > 0.5)) //check if the calculated root is within range (.5,2)
		  	{
		  		for (int j=0; j<6; j++) 
		  		{
		  			met_temp = met_temp + a[j] * pow(roots1[i],5-j);
		  		}
		  		if (met_temp > check) // if the root can return a metric that is greater than current metric
		  		{
		  			opt_gamma = roots1[i];
		  			// std::cout << "\n in function maximum metric is: " << met_temp << "\n" << std::endl;
		  		}
		  	}
		  // std::cout << "eig(i) is: " << std::real(eig (i)) << "   ima: " << std::imag (eig (i)) << std::endl;
		} // end of for loop with index i
		return opt_gamma;
	} // END of function of findRoots1()


	double * ExpNode::curveFit (double x[7], double y[7])
	{ 
		static double coff[6];
		int i, j, k, n, N;

		n = 5;

		Eigen::MatrixXd A(7,6);
		Eigen::MatrixXd b(7,1);

		for (i = 0; i <7; i++)
		for (j = 5; j>=0; j--)
		{A(i,5-j) = pow (x[i], j);}

		for (i = 0; i <7; i++)
		{  b(i,0) = y[i]; } 

		Eigen::MatrixXd A1 = A.transpose()*A;
		Eigen::MatrixXd b1 = A.transpose()*b; 
		//Eigen::MatrixXd Q =A1.colPivHouseholderQr().solve(b1);
		Eigen::MatrixXd Q =A1.inverse()*b1;

		for(i=0; i<n+1; i++)
		{  coff[i] = Q(i);
		//std::cout << "\nx is: " << x[i] << "Q is: " << coff[i] << std::endl;
		}

		return coff;
	   
	} // END of function curveFit()




    

} //END OF THE WHOLE NAMESPACE