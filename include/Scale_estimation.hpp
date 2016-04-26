double Scale(std::vector<double> X, std::vector<double> Y) 
{
    double sigmax = standard_deviation(X); //% 0.0834 ; %mean(std(X));
    double sigmay = 0.01;
    // standard_deviation(Y);// %0.0648 ; %mean(std(Y));

    // ROS_INFO_STREAM("sigmax: "<< sigmax);
    // ROS_INFO_STREAM("sigmay: "<< sigmay);


    int N = X.size();

    double sumxx = 0;
    double sumyy = 0;
    double sumxy = 0;

    for(int i = 0; i< N; i++ )
    sumxx = sumxx + (X[i]*X[i]);

    // ROS_INFO_STREAM("sumxx: "<< sumxx);

    double  sxx = std::pow(sigmay,2)*sumxx;

    // ROS_INFO_STREAM("sxx: "<< sxx);


    for(int i=0; i<N; i++)    
        sumyy = sumyy + (Y[i]*Y[i]);

    // ROS_INFO_STREAM("sumyy: "<< sumyy);

    double syy = std::pow(sigmay,2)*sumxx;

    // ROS_INFO_STREAM("syy: "<< syy);

    for (int i = 0; i< N; i++)
        sumxy = sumxy +(X[i]*Y[i]);

    // ROS_INFO_STREAM("sumxy: "<< sumxy);

    double sxy = sigmay*sigmax*sumxy;

    // ROS_INFO_STREAM("sxy: "<< sxy);

    double temp_sign;
    
    if(sxy == 0)
        temp_sign = 0;
    else
        temp_sign =  (sxy>0 ? 1 : -1);

    // ROS_INFO_STREAM("temp_sign: "<< temp_sign);

    // double S = (sxx-syy+temp_sign*std::sqrt(std::pow((sxx-syy),2)+4*sxy*sxy))/(2*(1/sigmax)*sigmay*sxy);
    // ROS_INFO_STREAM("S: "<< S);

    double scale =1/( (sxx-syy+temp_sign*std::sqrt(std::pow((sxx-syy),2)+4*sxy*sxy))/(2*(1/sigmax)*sigmay*sxy));
    // ROS_INFO_STREAM("scala: "<< scale);

    return scale;
}



double standard_deviation(std::vector<double> data)
{
    double mean=0.0;
    double sum_deviation=0.0;
    int n = data.size();

    for(int i=0; i < n; ++i)
    {
        mean+=data[i];
    }

    mean=mean/n;

    for(int i=0; i < n; ++i)
    sum_deviation+=(data[i]-mean)*(data[i]-mean);
    
    return std::sqrt(sum_deviation/n);           
}


double Scale_factor(std::vector<double> X, std::vector<double> Y) 
{

    double sigmax = standard_deviation(X); //% 0.0834 ; %mean(std(X));
    double sigmay = standard_deviation(Y);// %0.0648 ; %mean(std(Y));

    return ( (sigmay*sigmay) /(sigmax*sigmax) );
}


double ScalaReturn(double ptam, double ptam_prev, double robot)
{
    double scala_temp = 0;
    
    if(ptam !=0 )
    {
        scala_temp = robot/ptam;
    }
    // ROS_INFO_STREAM("scala: " << scala_temp);
    return scala_temp;
}
