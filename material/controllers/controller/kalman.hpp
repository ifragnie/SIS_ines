
#include <math.h>
#include <memory.h>
#include <Eigen/Dense>

#include "odometry.hpp"

///////////////////
// Eigen library //     DO NOT MODIFY THIS PART
///////////////////

#include <Eigen/Dense>

#define DIM 3                                       // State dimension 

typedef Eigen::Matrix<double,DIM,DIM>   Mat;        // DIMxDIM matrix  
typedef Eigen::Matrix<double, -1, -1>   MatX;       // Arbitrary size matrix 
typedef Eigen::Matrix<double,DIM,  1>   Vec;        // DIMx1 column vector  
typedef Eigen::Matrix<double, -1,  1>   VecX;       // Arbitrary size column vector  

static const Mat I = MatX::Identity(DIM,DIM);       // DIMxDIM identity matrix  

//////////////////////////////////
// Kalman filter base functions //    DO NOT MODIFY THIS PART
//////////////////////////////////

// State vector mu (x,y,heading) to be updated by the Kalman filter functions
static Vec mu = Vec::Zero();
// State covariance sigma to be updated by the Kalman filter functions
static Mat sigma = Mat::Zero();

static bool near_node = false;
/**bon
 * @brief      Get the state dimension 
*/
int kal_get_dim(){
    return DIM;
}

/**
 * @brief      Copy the state vector into a 1D array
*/
void kal_get_state(double* state){
    for(int i=0; i<DIM; i++){
        state[i] = mu(i);
    }
}

/**
 * @brief      Copy the state covariance matrix into a 2D array
*/
void kal_get_state_covariance(double** cov){
    for(int i=0;i<sigma.rows();i++){
        for(int j=0; j<sigma.cols(); j++){
            cov[i][j] = sigma(i,j);
        }
    }
}

/**
 * @brief      Check if a matrix contains any NaN values 
*/
bool kal_check_nan(const MatX& m){
    for(int i=0;i<m.rows();i++){
        for(int j=0; j<m.cols(); j++){
            if(isnan(m(i,j))){
                printf("FATAL: matrix has NaN values, exiting...\n");
                return true;
            }
        }
    }
    return false;
}

void kal_set_node_visibility(bool visible)
{
    near_node = visible;
}

static Vec mu_pred = Vec::Zero();
static Mat sigma_pred = Mat::Identity();


///////////////////////////////////////////////////
// TODO: implement your Kalman filter here after //

/*KALMAN PREDICTION QUI MARCHE EN ATTENTE DE MIEUX
void kal_prediction(double v , double odo_w, double dt, bool close_to_node ){

    double theta = mu(2);

    // state preidction
    //Vec new_mu; 
    
    //ENLEVER SI PB !! VRAIMENT PAS SUR DE CETTE CHOSE LOL
    //new_mu(2)=atan2(sin(new_mu(2)), cos(new_mu(2))); 

    //Jacobian for linearization 
    

    //Rotational matrix from local to global frame
    Mat T=Mat::Identity();
    T(0,0)=cos(theta);
    T(0,1)=-sin(theta);
    T(1,0)=sin(theta);
    T(1,1)=cos(theta);
    Mat F = Mat::Identity();
    Mat Sigma_u=Mat::Zero();
    Mat R = Mat::Identity();
    //covriance matrix of velocities
    //std
    double sigma_v;//=0.05;//0.05
    double sigma_w;//=0.25; //0.25
    
    
    if(close_to_node == true){
        // robot close to node
      mu_pred(0)=mu(0)+cos(theta)*v*dt;
      mu_pred(1)=mu(1)+sin(theta)*v*dt;
     //mu_pred(2)=gyro_w;
      mu_pred(2)=mu(2)+odo_w*dt;
      mu_pred(2)=atan2(sin(mu_pred(2)), cos(mu_pred(2)));
        
      sigma_v = 0.02;//changer ?
      sigma_w = 0.015;//changer ?
        
        
      F(0,2)=-v*sin(theta)*dt;
      F(1,2)=v*cos(theta)*dt;
        
        
      Sigma_u(0,0)=sigma_v*sigma_v*dt*dt;
      Sigma_u(1,1)=0;//sigma_v*sigma_v*dt*dt;
      Sigma_u(2,2)=sigma_w*sigma_w*dt*dt;
    
    

          //noise matrix
       R = T*Sigma_u*T.transpose();

    
    }else{
     

    // state preidction
    //Vec new_mu; 
    
    //ENLEVER SI PB !! VRAIMENT PAS SUR DE CETTE CHOSE LOL
    //new_mu(2)=atan2(sin(new_mu(2)), cos(new_mu(2))); 

    //Jacobian for linearization 
    

    //Rotational matrix from local to global frame
      
        // robot close to node
      mu_pred(0)=mu(0)+cos(theta)*v*dt;
      mu_pred(1)=mu(1)+sin(theta)*v*dt;
      mu_pred(2)=mu(2)+odo_w*dt;
      mu_pred(2)=atan2(sin(mu_pred(2)), cos(mu_pred(2)));
        
      sigma_v = 0.05;//changer ?
      sigma_w = 0.025;//changer ?
        
        
      F(0,2)=-v*sin(theta)*dt;
      F(1,2)=v*cos(theta)*dt;
        
        
      Sigma_u(0,0)=sigma_v*sigma_v*dt*dt;
      Sigma_u(1,1)=0;//sigma_v*sigma_v*dt*dt;
      Sigma_u(2,2)=sigma_w*sigma_w*dt*dt;
    
    

          //noise matrix
       R = T*Sigma_u*T.transpose(); 
        //double vx = acc_x * dt;
        //double vy = acc_y * dt;

    //////////////////////////////////////////////////
    // control vector u
    //////////////////////////////////////////////////

        //Vec u;

        //u(0) = vx;
        //u(1) = vy;
        //u(2) = gyro_w;
        
       // Mat B = Mat::Zero();

       // B(0,0) = dt*cos(theta);
       // B(0,1) = -dt*sin(theta);

        //B(1,0) = dt*sin(theta);
        //B(1,1) = dt*cos(theta);

       // B(2,2) = dt;
        // odometry only
       // double sigma_acc = 0.05;
       // double sigma_gyro = 0.025;
       // double sigma_v=sigma_acc*dt;
        
        
        
       //mu_pred=F*mu+B*u;
        
        //Sigma_u(0,0) = sigma_v * sigma_v;
       // Sigma_u(1,1) = sigma_v * sigma_v;
       // Sigma_u(2,2) = sigma_gyro * sigma_gyro;
        
        //R=B*Sigma_u*B.transpose();
     
    }
    
    

    //Covariance update 
     sigma_pred=F*sigma*F.transpose()+R; // averififier si sigma_u ou sigma ??


    //update the state
     mu=mu_pred;    
     sigma=sigma_pred;
}
*/


void kal_prediction(double distL, double distR, double gyro_z, double dt, bool close_to_node) {
    const double L = PioneerInfo::axis_length;

    double dCenter = (distR + distL) / 2.0;
    
    // Rotation : gyroscope au lieu des encodeurs
    double dTheta = gyro_z * dt;

    double theta_mid = mu(2) + dTheta / 2.0;

    mu_pred(0) = mu(0) + dCenter * cos(theta_mid);
    mu_pred(1) = mu(1) + dCenter * sin(theta_mid);
    mu_pred(2) = mu(2) + dTheta;
    mu_pred(2) = atan2(sin(mu_pred(2)), cos(mu_pred(2)));

    double sigma_v, sigma_w;
    if (close_to_node) {
        sigma_v = 0.02;
        sigma_w = 0.015;
    } else {
        sigma_v = 0.05;
        sigma_w = 0.025;
    }

    Mat F = Mat::Identity();
    F(0,2) = -dCenter * sin(theta_mid);
    F(1,2) =  dCenter * cos(theta_mid);

    Mat R = Mat::Zero();
    R(0,0) = sigma_v * sigma_v;
    R(1,1) = sigma_v * sigma_v;
    R(2,2) = sigma_w * sigma_w;

    sigma_pred = F * sigma * F.transpose() + R;
    mu    = mu_pred;
    sigma = sigma_pred;
}

//void kal_update_headig(double heading_meas)
///////////////////////////////////////////////////
void kal_update_position(double x_node, double y_node, double distance){


    //measurment matrix
    Eigen::Matrix<double,2,3> H;
    H<< 1,0,0,
        0,1,0;

    //measurment covariance matrix
    //double k=1;
    //double sigma_pos=0.25;//k*distance*distance;

   
    
    double sigma_pos;
    double sigma_pos_x;
    double sigma_pos_y;
        
    if(distance < 1.0)
    {
        sigma_pos_x = 0.1;
        sigma_pos_y=0.2;
    }
    else
    {
        sigma_pos = 0.02;
    }
   


    Eigen::Matrix<double,2,2> Q;

    Q<<sigma_pos*sigma_pos, 0,
        0,sigma_pos*sigma_pos;
        

    //measurment vector 
    Eigen::Matrix<double,2,1> innovation;
    
    
    
    //double alpha = 0.05;
     Eigen::Matrix<double,2,1> z;
    
     z << x_node, //- mu_pred(0),
          y_node; //- mu_pred(1);
    //z<<x_node, 
        //y_node;

    //predicted measurment(odometry)
     Eigen::Matrix<double,2,1> z_pred;

     z_pred << mu_pred(0),
               mu_pred(1);
              
              
              
      
    //////////////////////////////////////////////////
    // Innovation
    //////////////////////////////////////////////////

      

     innovation = z - z_pred;
    //}else{
      //pred_heading=gyro_w;
      //true_heading=imu[5];
      //innovation << mu_pred(0),
        //            mu_pred(1);
    
     //update
    Eigen::Matrix<double,2,2> S;
    S= H*sigma_pred*H.transpose()+Q;
    if(fabs(S.determinant()) < 1e-9){ 
    return;
    }
    Eigen::Matrix<double,3,2> K;
    K = sigma_pred * H.transpose() * S.inverse();
    
    
    
    mu=mu_pred+K*innovation;
    sigma=(I-K*H)*sigma_pred;
    if (fabs(mu(2)) < M_PI / 2.0) {
        mu(2) = 0.0;        // va vers +x  ✓ déjà bon
    } else {
        
        mu(2) = (mu(2) > 0) ? M_PI : -M_PI;
    }

}