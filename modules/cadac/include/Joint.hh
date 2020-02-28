#ifndef JOINT_HPP
#define JOINT_HPP
#include <armadillo>
#include <memory>
#include "Body.hh"
#include "matrix_tool.hh"


class Joint
{
public:
    Joint(unsigned int TypeIn, arma::vec piIn, arma::vec pjIn, arma::vec qiIn,
            arma::vec qjIn, Body *i_In, Body *j_In);
    ~Joint() {};
    void Build_C();
    void Build_Cq();
    void Build_GAMMA();
    void update();

    arma::mat get_Cqi();
    arma::mat get_Cqj();
    arma::vec get_GAMMA();
    arma::vec get_Pi();
    arma::vec get_Pj();
    arma::vec get_pi();
    arma::vec get_pj();
    arma::vec get_CONSTRAINT();
    Body* get_body_i_ptr();
    Body* get_body_j_ptr();

private:
    unsigned int Type;  
    arma::vec pi;
    arma::vec pj;
    arma::vec qi;
    arma::vec qj;
    arma::mat Cqi;
    arma::mat Cqj;
    arma::vec GAMMA;
    arma::vec CONSTRAINT;
    arma::mat TBI_i;
    arma::mat TBI_j;
    arma::vec Pi;
    arma::vec Pj;
    arma::vec Qi;
    arma::vec Qj;
    arma::vec wi;
    arma::vec wj;
    arma::vec Si;
    arma::vec Sj;
    Body *body_i_ptr;
    Body *body_j_ptr;
};
#endif  //JOINT_HPP