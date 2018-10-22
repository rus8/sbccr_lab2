#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpFeaturePoint.h>
#include <ecn_sensorbased/optim.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    PioneerCam robot;

    // gains
    // pose error gain
    double lv = .5;
    geometry_msgs::Pose2D target;

    int it = 0;
    // variables
    vpColVector r(4); // reference vector
    vpColVector q(4); // input vector
    vpColVector s_ball(2); // feature - ball pose in image
    vpFeaturePoint s;
    vpMatrix L_x(2, 4);
    vpMatrix J_vq(6, 4);

    // params
    double alpha = 0.1;
    double dist = 0.1;
    double wheel_rad = robot.radius();
    double base = robot.base();
    double w_max = robot.wmax();
    
    // optimization stuff
    vpMatrix Q; // minimization problem matrix
    Q.eye(4);

    // inequality constraints params
    vpMatrix C_tot(8, 4);
    vpColVector d_tot(8);

    // wheels speed
    vpMatrix C_w(4, 4);
    C_w[0][0] = 1/wheel_rad; C_w[0][1] = base/wheel_rad;
    C_w[1][0] = -1/wheel_rad; C_w[1][1] = -base/wheel_rad;
    C_w[2][0] = 1/wheel_rad; C_w[2][1] = -base/wheel_rad;
    C_w[3][0] = -1/wheel_rad; C_w[3][1] = base/wheel_rad;
    vpColVector d_w(4);
    d_w[0] = w_max; d_w[1] = w_max; d_w[2] = w_max; d_w[3] = w_max;
    
    // visibility
    vpMatrix C_v(4, 4);
    vpColVector d_v(4);
    vpColVector cam_lim(2);
    cam_lim = robot.getCamLimits();


    // equality constraints params
    vpMatrix A_t(1, 4);
    vpColVector b_t(1);

    while(ros::ok())
    {
        it++;
        cout << "-------------" << endl;

        if(robot.ok())
        {
            // get robot and target positions to get position error
            target = robot.getTargetRelativePose();

            // linear velocity
            r[0] = lv*(target.x - dist);
            // angular velocity
            r[1] = 10*lv*(fmod(atan2(target.y, target.x)+M_PI, 2*M_PI) - M_PI);

            // Creation of the current feature s
            robot.getImagePoint(s_ball); // get ball pose
            s.buildFrom(s_ball[0], s_ball[1], 1);
            L_x = s.interaction( vpBasicFeature::FEATURE_ALL );
            J_vq = robot.getCamJacobian();

            ecn::putAt(C_v, L_x * J_vq, 0, 0);
            ecn::putAt(C_v, L_x * J_vq, 2, 0);
            ecn::putAt(d_v, (cam_lim - s_ball) * alpha, 0);
            ecn::putAt(d_v, -(-cam_lim - s_ball) * alpha, 2);

            A_t[0][0] = r[1]; A_t[0][1] = -r[0];

            ecn::putAt(C_tot, C_w, 0, 0);
            ecn::putAt(C_tot, C_v, 4, 0);
            ecn::putAt(d_tot, d_w, 0);
            ecn::putAt(d_tot, d_v, 4);

            cout << "ref: " << r.t() << endl;

            ecn::solveQP(Q, r, A_t, b_t, C_tot, d_tot, q);

            robot.setVelocity(q);
        }
    }
}
