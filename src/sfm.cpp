/*
	July 9, 2020, He Zhang, hzhang8@vcu.edu 

	sfm, do it 

*/

#include "sfm.h"

GlobalSFM::GlobalSFM(){}

vector<SFMFeature> GlobalSFM::init_feat_obs(vector<std::vector<feat_obs> >& all_obs, Quaterniond q0, Vector3d t0, double sigma)
{
	static double FOCAL_LENGTH = 460.; 
	std::normal_distribution<double> noise(0.0, sigma/FOCAL_LENGTH);
	std::random_device rd;
    std::default_random_engine generator_(rd());
	// construct data 
	int M = all_obs[0].size();  // number of features be tracked 
	vector<SFMFeature> v_sfm_feats(M); 
	for(int j=0; j < M; j++){

		SFMFeature& sfm_feat = v_sfm_feats[j]; 

		sfm_feat.state = true; 
		sfm_feat.id = j; 

		// depth is set as the value in first frame 
		sfm_feat.depth = all_obs[0][j].depth; 

		for(int i=0; i<all_obs.size(); i++){
			// add noise to u, v,  
			Vector2d uv; 
			uv[0] = all_obs[i][j].uv[0] + noise(generator_); 
			uv[1] = all_obs[i][j].uv[1] + noise(generator_); 
			sfm_feat.observation.emplace_back(make_pair(i, Vector2d(all_obs[i][j].uv[0], all_obs[i][j].uv[1])));
			sfm_feat.observation_depth.emplace_back(make_pair(i, all_obs[i][j].depth));
			if(i==0){
				// feature position 
				Vector3d feat_in_cam(uv[0]*sfm_feat.depth, uv[1]*sfm_feat.depth, sfm_feat.depth); 
				Vector3d feat_in_world = q0 * feat_in_cam + t0; 
				sfm_feat.position[0] = feat_in_world[0]; 
				sfm_feat.position[1] = feat_in_world[1]; 
				sfm_feat.position[2] = feat_in_world[2]; 
			}
		}
	}
	return v_sfm_feats;
}

double GlobalSFM::do_it(int N, vector<std::vector<feat_obs> >& all_obs, std::vector<cam_pose>* p_all_pose, double sigma)
{
	// first pose 
	Quaterniond q0(1, 0, 0, 0); 
	Vector3d t0(0, 0, 0); 
	if(p_all_pose!=NULL){
		cam_pose& gt_p = (*p_all_pose)[0];
		q0 = Quaterniond(gt_p.qw, gt_p.qx, gt_p.qy, gt_p.qz); 
		t0 = Vector3d(gt_p.x, gt_p.y, gt_p.z); 
	}
	int M = all_obs[0].size();  // number of features be tracked 

	vector<SFMFeature> v_sfm_feats = init_feat_obs(all_obs, q0, t0, sigma); 
	// sfm, gogogo 
	//full BA
	double (*c_translation)[3] = new double[N][3]; 
	double (*c_rotation)[4] = new double[N][4]; 

	Matrix3d R_0 = Matrix3d::Identity(); 
	Vector3d t_0 = Vector3d::Zero(); 
	solveFrameByPnP(R_0, t_0, 0, v_sfm_feats); 

	// Eigen::Vector3d Ps[WN+1]; 
    // Eigen::Vector3d Vs[WN+1]; 
    // Eigen::Matrix3d Rs[WN+1]; 
    // Matrix3d tmpR = R_0.transpose();
    // Vector3d tmpt = -tmpR * t_0; 
	Quaterniond q_0(R_0); // tmpR R_0
	// t_0 = tmpt; 
	c_rotation[0][0] = q0.w();  
	c_rotation[0][1] = q0.x(); 
	c_rotation[0][2] = q0.y(); 
	c_rotation[0][3] = q0.z();
	c_translation[0][0] = t0.x(); 
	c_translation[0][1] = t0.y(); 
	c_translation[0][2] = t0.z(); 

	/*cout <<" q_0: "<<q_0.w()<<" "<<q_0.x()<<" "<<q_0.y()<<" "<<q_0.z()<<" t_0: "<<t_0.transpose()<<endl; 
	Quaterniond q_1 = q_0.inverse(); // (R_0.transpose()); 
	cout <<" q_1: "<<q_1.w()<<" "<<q_1.x()<<" "<<q_1.y()<<" "<<q_1.z()<<endl; 
	Quaterniond q_2 (R_0.transpose()); 
	cout <<" q_2: "<<q_2.w()<<" "<<q_2.x()<<" "<<q_2.y()<<" "<<q_2.z()<<endl; 

	cout<<"R_1: "<<q_1.toRotationMatrix()<<endl << 
		"R_2: "<<q_2.toRotationMatrix()<<endl;
	Quaterniond dq = q_1.inverse()*q_2; 
	cout<<"dq: "<<dq.w()<<" "<<dq.vec().transpose()<<endl;
*/
	// initilization 
	
	for(int i=0; i<N; i++){
/*		c_translation[i][0] = c_translation[i][1] = c_translation[i][2] = 0; 
		c_rotation[i][1] = c_rotation[i][2] = c_rotation[i][3] = 0;
		c_rotation[i][0] =1.;  */
		c_translation[i][0] = t_0.x();
		c_translation[i][1] = t_0.y();
		c_translation[i][2] = t_0.z();
		c_rotation[i][1] = q_0.x(); c_rotation[i][2] = q_0.y(); c_rotation[i][3] = q_0.z();
		c_rotation[i][0] = q_0.w();
	}


	ceres::Problem problem;
	ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
	//cout << " begin full BA " << endl;
	for (int i = 0; i < N; i++)
	{
		problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
		problem.AddParameterBlock(c_translation[i], 3);
		if (i == 0)
		{
			problem.SetParameterBlockConstant(c_rotation[i]);
			problem.SetParameterBlockConstant(c_translation[i]);
		}
	}

	vector<SFMFeature>& sfm_f = v_sfm_feats;  
	for (int i = 0; i < M; i++)
	{
		if (sfm_f[i].state != true)
			continue;
		for (int j = 0; j < int(sfm_f[i].observation.size()); j++)
		{
			int l = sfm_f[i].observation[j].first;
			ceres::CostFunction* cost_function = ReprojectionError3D::Create(
												sfm_f[i].observation[j].second.x(),
												sfm_f[i].observation[j].second.y());

    		ceres::ResidualBlockId param_id = problem.AddResidualBlock(cost_function, NULL, c_rotation[l], c_translation[l], 
    								sfm_f[i].position);	 
    		{
    			vector<double*>* para = new vector<double*>;  
                problem.GetParameterBlocksForResidualBlock(param_id, para); 
                vector<double> res(2); 
                cost_function->Evaluate(&para[0][0], &res[0], 0); 
                if(res[0] != res[0]){
                	cout<<"i: "<<i<<" j: "<<j<<" (u,v): "<<sfm_f[i].observation[j].second.x()<<" "<<sfm_f[i].observation[j].second.y()
                	<< " position: "<<sfm_f[i].position[0]<<" "<<sfm_f[i].position[1]<<" "<<sfm_f[i].position[2]<<endl;
                	cout<<"dvio.cpp: residual: "<<res[0]<<" "<<res[1]<<endl;
            	}
    		}	
		}

	}
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	// options.minimizer_progress_to_stdout = true;
	options.max_solver_time_in_seconds = 0.2;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	// std::cout << summary.BriefReport() << "\n";
	if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03)
	{
		// cout << "initial_sfm.cpp: vision only BA converge" << endl;
	}
	else
	{
		cout << "initial_sfm.cpp: vision only BA not converge " << endl;
		return 0;
	}

	vector<Quaterniond> q(N); 
	for (int i = 0; i < N; i++)
	{
		q[i].w() = c_rotation[i][0]; 
		q[i].x() = c_rotation[i][1]; 
		q[i].y() = c_rotation[i][2]; 
		q[i].z() = c_rotation[i][3]; 
		q[i] = q[i].inverse();
		// cout << "final  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
	}
	vector<Vector3d> T(N); 
	for (int i = 0; i < N; i++)
	{

		T[i] = -1 * (q[i] * Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
		// T[i] = Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]);
		// cout << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"  "<< T[i](2) << endl;
	}

	// compute rmse 
	if(p_all_pose != NULL && N > 0){
		double rmse = 0; 
		for(int i=0; i<N; i++){
			cam_pose& gt_p = (*p_all_pose)[i];
			rmse = rmse + SQ(gt_p.x - T[i](0)) + SQ(gt_p.y - T[i](1)) + SQ(gt_p.z - T[i](2)); 
		}
		rmse = sqrt(rmse/N); 
		cout <<" sfm.cpp: rmse: "<<rmse<<endl; 
		return rmse;
	}
	return 0;
}

bool GlobalSFM::solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i,
								vector<SFMFeature> &sfm_f)
{
	vector<cv::Point2f> pts_2_vector;
	vector<cv::Point3f> pts_3_vector;
	for (int j = 0; j < sfm_f.size(); j++)
	{
		if (sfm_f[j].state != true)
			continue;
		Vector2d point2d;
		for (int k = 0; k < (int)sfm_f[j].observation.size(); k++)
		{
			if (sfm_f[j].observation[k].first == i)
			{
				Vector2d img_pts = sfm_f[j].observation[k].second;
				cv::Point2f pts_2(img_pts(0), img_pts(1));
				pts_2_vector.push_back(pts_2);
				cv::Point3f pts_3(sfm_f[j].position[0], sfm_f[j].position[1], sfm_f[j].position[2]);
				pts_3_vector.push_back(pts_3);
				break;
			}
		}
	}
	// printf("at frame %d has observations: %d\n", i, pts_2_vector.size());
	if (int(pts_2_vector.size()) < 15)
	{
		printf("initial_sfm.cpp: for frame %d, only %d features are found \
			unstable features tracking, please slowly move you device!\n", i, pts_2_vector.size());
		if (int(pts_2_vector.size()) < 10){
			printf("initial_sfm.cpp: return false since only %d features are found", pts_2_vector.size());
			return false;
		}
	}
	cv::Mat r, rvec, t, D, tmp_r;
	cv::eigen2cv(R_initial, tmp_r);
	cv::Rodrigues(tmp_r, rvec);
	cv::eigen2cv(P_initial, t);
	cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	bool pnp_succ;
	pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 0);
	if(!pnp_succ)
	{
		return false;
	}
	cv::Rodrigues(rvec, r);
	//cout << "r " << endl << r << endl;
	MatrixXd R_pnp;
	cv::cv2eigen(r, R_pnp);
	MatrixXd T_pnp;
	cv::cv2eigen(t, T_pnp);
	R_initial = R_pnp;
	P_initial = T_pnp;
	return true;

}


// 	 q w_R_cam t w_R_cam
//  c_rotation cam_R_w 
//  c_translation cam_R_w
// relative_q[i][j]  j_q_i
// relative_t[i][j]  j_t_ji  (j < i)
/*bool GlobalSFM::construct(int frame_num, Quaterniond* q, Vector3d* T, int l,
			  const Matrix3d relative_R, const Vector3d relative_T,
			  vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points)
{
	feature_num = sfm_f.size();
	//cout << "set 0 and " << l << " as known " << endl;
	// have relative_r relative_t
	// intial two view
	q[l].w() = 1;
	q[l].x() = 0;
	q[l].y() = 0;
	q[l].z() = 0;
	T[l].setZero();
	q[frame_num - 1] = q[l] * Quaterniond(relative_R);
	T[frame_num - 1] = relative_T;
	//cout << "init q_l " << q[l].w() << " " << q[l].vec().transpose() << endl;
	//cout << "init t_l " << T[l].transpose() << endl;

	//rotate to cam frame
	Matrix3d c_Rotation[frame_num];
	Vector3d c_Translation[frame_num];
	Quaterniond c_Quat[frame_num];
	double c_rotation[frame_num][4];
	double c_translation[frame_num][3];
	Eigen::Matrix<double, 3, 4> Pose[frame_num];

	c_Quat[l] = q[l].inverse();
	c_Rotation[l] = c_Quat[l].toRotationMatrix();
	c_Translation[l] = -1 * (c_Rotation[l] * T[l]);
	Pose[l].block<3, 3>(0, 0) = c_Rotation[l];
	Pose[l].block<3, 1>(0, 3) = c_Translation[l];

	c_Quat[frame_num - 1] = q[frame_num - 1].inverse();
	c_Rotation[frame_num - 1] = c_Quat[frame_num - 1].toRotationMatrix();
	c_Translation[frame_num - 1] = -1 * (c_Rotation[frame_num - 1] * T[frame_num - 1]);
	Pose[frame_num - 1].block<3, 3>(0, 0) = c_Rotation[frame_num - 1];
	Pose[frame_num - 1].block<3, 1>(0, 3) = c_Translation[frame_num - 1];


	//1: trangulate between l ----- frame_num - 1
	//2: solve pnp l + 1; trangulate l + 1 ------- frame_num - 1; 
	for (int i = l; i < frame_num - 1 ; i++)
	{
		// solve pnp
		if (i > l)
		{
			Matrix3d R_initial = c_Rotation[i - 1];
			Vector3d P_initial = c_Translation[i - 1];
			if(!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
				return false;
			c_Rotation[i] = R_initial;
			c_Translation[i] = P_initial;
			c_Quat[i] = c_Rotation[i];
			Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
			Pose[i].block<3, 1>(0, 3) = c_Translation[i];
		}

		// triangulate point based on the solve pnp result
		triangulateTwoFramesWithDepth(i, Pose[i], frame_num - 1, Pose[frame_num - 1], sfm_f);
		triangulateTwoFrames(i, Pose[i], frame_num - 1, Pose[frame_num - 1], sfm_f)	;
	}
	//3: triangulate l-----l+1 l+2 ... frame_num -2
	for (int i = l + 1; i < frame_num - 1; i++){
		triangulateTwoFramesWithDepth(l, Pose[l], i, Pose[i], sfm_f);
		triangulateTwoFrames(l, Pose[l], i, Pose[i], sfm_f);
	}

	//4: solve pnp l-1; triangulate l-1 ----- l
	//             l-2              l-2 ----- l
	for (int i = l - 1; i >= 0; i--)
	{
		//solve pnp
		Matrix3d R_initial = c_Rotation[i + 1];
		Vector3d P_initial = c_Translation[i + 1];
		if(!solveFrameByPnP(R_initial, P_initial, i, sfm_f))
			return false;
		c_Rotation[i] = R_initial;
		c_Translation[i] = P_initial;
		c_Quat[i] = c_Rotation[i];
		Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
		Pose[i].block<3, 1>(0, 3) = c_Translation[i];
		//triangulate
		triangulateTwoFramesWithDepth(i, Pose[i],  l, Pose[l], sfm_f);
		triangulateTwoFrames(i, Pose[i], l, Pose[l], sfm_f);
	}
	//5: triangulate all other points
	// for (int j = 0; j < feature_num; j++)
	// {
	// 	if (sfm_f[j].state == true)
	// 		continue;
	// 	if ((int)sfm_f[j].observation.size() >= 2)
	// 	{
	// 		Vector2d point0, point1;
	// 		int frame_0 = sfm_f[j].observation[0].first;
	// 		point0 = sfm_f[j].observation[0].second;
	// 		int frame_1 = sfm_f[j].observation.back().first;
	// 		point1 = sfm_f[j].observation.back().second;
	// 		Vector3d point_3d;
	// 		triangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1, point_3d);
	// 		sfm_f[j].state = true;
	// 		sfm_f[j].position[0] = point_3d(0);
	// 		sfm_f[j].position[1] = point_3d(1);
	// 		sfm_f[j].position[2] = point_3d(2);
	// 		//cout << "trangulated : " << frame_0 << " " << frame_1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
	// 	}		
	// }

	//5: triangulate all other points
	for (int j = 0; j < feature_num; j++)
	{
		if (sfm_f[j].state == true)
			continue;
		if ((int)sfm_f[j].observation.size() >= 2)
		{
			Vector3d point0; 
			Vector2d point1;
			int frame_0 = sfm_f[j].observation[0].first;
			double depth_i = sfm_f[j].observation_depth[0].second; 

			if(depth_i <= 0.1)
				continue; // not a valid point 


			point0 = Vector3d(sfm_f[j].observation[0].second.x()*depth_i,sfm_f[j].observation[0].second.y()*depth_i,depth_i);
			int frame_1 = sfm_f[j].observation.back().first;
			point1 = sfm_f[j].observation.back().second;
			Vector3d point_3d;
			//triangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1, point_3d);

			Matrix3d Pose0_R = Pose[frame_0].block< 3,3 >(0,0);
			Matrix3d Pose1_R = Pose[frame_1].block< 3,3 >(0,0);
			Vector3d Pose0_t = Pose[frame_0].block< 3,1 >(0,3);
			Vector3d Pose1_t = Pose[frame_1].block< 3,1 >(0,3);

			Vector2d residual;
			Vector3d point1_reprojected;
			//triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
			point_3d = Pose0_R.transpose()*point0 - Pose0_R.transpose()*Pose0_t;//point in world;
			point1_reprojected = Pose1_R*point_3d+Pose1_t;

			if(point1_reprojected.z()<0.1){
				residual = Vector2d(10,10);
			}else
				residual = point1 - Vector2d(point1_reprojected.x()/point1_reprojected.z(),point1_reprojected.y()/point1_reprojected.z());

			if (residual.norm() < 1.0/460) {//reprojection error
				sfm_f[j].state = true;
				sfm_f[j].position[0] = point_3d(0);
				sfm_f[j].position[1] = point_3d(1);
				sfm_f[j].position[2] = point_3d(2);
			}
			//cout << "trangulated : " << frame_0 << " " << frame_1 << "  3d point : "  << j << "  " << point_3d.transpose() << endl;
		}		
	}


	//full BA
	ceres::Problem problem;
	ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
	//cout << " begin full BA " << endl;
	for (int i = 0; i < frame_num; i++)
	{
		//double array for ceres
		c_translation[i][0] = c_Translation[i].x();
		c_translation[i][1] = c_Translation[i].y();
		c_translation[i][2] = c_Translation[i].z();
		c_rotation[i][0] = c_Quat[i].w();
		c_rotation[i][1] = c_Quat[i].x();
		c_rotation[i][2] = c_Quat[i].y();
		c_rotation[i][3] = c_Quat[i].z();
		problem.AddParameterBlock(c_rotation[i], 4, local_parameterization);
		problem.AddParameterBlock(c_translation[i], 3);
		if (i == l)
		{
			problem.SetParameterBlockConstant(c_rotation[i]);
		}
		if (i == l || i == frame_num - 1)
		{
			problem.SetParameterBlockConstant(c_translation[i]);
		}
	}

	for (int i = 0; i < feature_num; i++)
	{
		if (sfm_f[i].state != true)
			continue;
		for (int j = 0; j < int(sfm_f[i].observation.size()); j++)
		{
			int l = sfm_f[i].observation[j].first;
			ceres::CostFunction* cost_function = ReprojectionError3D::Create(
												sfm_f[i].observation[j].second.x(),
												sfm_f[i].observation[j].second.y());

    		problem.AddResidualBlock(cost_function, NULL, c_rotation[l], c_translation[l], 
    								sfm_f[i].position);	 
		}

	}
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	// options.minimizer_progress_to_stdout = true;
	options.max_solver_time_in_seconds = 0.2;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	// std::cout << summary.BriefReport() << "\n";
	if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03)
	{
		cout << "initial_sfm.cpp: vision only BA converge" << endl;
	}
	else
	{
		cout << "initial_sfm.cpp: vision only BA not converge " << endl;
		return false;
	}
	for (int i = 0; i < frame_num; i++)
	{
		q[i].w() = c_rotation[i][0]; 
		q[i].x() = c_rotation[i][1]; 
		q[i].y() = c_rotation[i][2]; 
		q[i].z() = c_rotation[i][3]; 
		q[i] = q[i].inverse();
		//cout << "final  q" << " i " << i <<"  " <<q[i].w() << "  " << q[i].vec().transpose() << endl;
	}
	for (int i = 0; i < frame_num; i++)
	{

		T[i] = -1 * (q[i] * Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
		//cout << "final  t" << " i " << i <<"  " << T[i](0) <<"  "<< T[i](1) <<"  "<< T[i](2) << endl;
	}

	return true;

}*/

