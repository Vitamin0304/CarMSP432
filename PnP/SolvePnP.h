/*
 * SolvePnP.h
 *
 *  Created on: 2021年3月16日
 *      Author: 电脑
 */

#ifndef PNP_SOLVEPNP_H_
#define PNP_SOLVEPNP_H_

void SolvePnP(Eigen::Matrix<double,3,3> & pixel,Eigen::Vector2d & F_Position);

#endif /* PNP_SOLVEPNP_H_ */
