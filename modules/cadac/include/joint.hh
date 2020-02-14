#ifndef JOINT_HPP
#define JOINT_HPP
#include <armadillo>
#include "aux.hh"
#include "body.hh"
#include "matrix_tool.hh"

class Joint
{
public:
    Joint(unsigned int TypeIn, arma::vec piIn, arma::vec pjIn, arma::vec qiIn,
          arma::vec qjIn, Body *i_In, Body *j_In);
    ~Joint(){};
    void Build_C();
    void Build_Cq();
    void Build_GAMMA();
    void update();

    arma::mat get_Cqi();
    arma::mat get_Cqj();
    arma::vec get_GAMMA();
    arma::vec get_Pi();
    arma::vec get_Pj();
    arma::vec get_CONSTRAINT();
    Body *get_body_i_ptr();
    Body *get_body_j_ptr();

private:
    unsigned int Type;
    VECTOR(pi, 3);
    VECTOR(pj, 3);
    VECTOR(qi, 3);
    VECTOR(qj, 3);
    MATRIX(Cqi, 3, 6);
    MATRIX(Cqj, 3, 6);
    VECTOR(GAMMA, 3);
    VECTOR(CONSTRAINT, 3);
    MATRIX(TBI_i, 3, 3);
    MATRIX(TBI_j, 3, 3);
    VECTOR(Pi, 3);
    VECTOR(Pj, 3);
    VECTOR(Qi, 3);
    VECTOR(Qj, 3);
    VECTOR(wi, 3);
    VECTOR(wj, 3);
    VECTOR(Si, 3);
    VECTOR(Sj, 3);
    Body *body_i_ptr;
    Body *body_j_ptr;
};

#endif  //JOINT_HPP