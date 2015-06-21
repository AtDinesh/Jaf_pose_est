#ifndef MUNKRES_H
#define MUNKRES_H

#include <eigen3/Eigen/Dense>


/**
 * @brief Solves the association problem using the Hungarian (aka Kuhn-Munkres) method
 * @param costs : matrix(i,j) of costs to associate task j to worker i
 * @param detections : filled by the procedure with 1 where i and j were masked and 0 elsewhere
 */
void Munkres(const Eigen::MatrixXd &costs, Eigen::MatrixXd &result);

#endif // MUNKRES_H
