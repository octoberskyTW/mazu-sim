#ifndef BODY_HPP
#define BODY_HPP

#include <armadillo>
#include "aux.hh"
#include "matrix_tool.hh"

class Body
{
public:
    Body();
    ~Body(){};

    arma::mat get_TBI();
    arma::vec get_POSITION();
    arma::vec get_VELOCITY();
    arma::vec get_ACCELERATION();
    arma::vec get_ANGLE_VEL();
    arma::vec get_ANGLE();
    arma::vec get_ANGLE_ACC();
    arma::vec get_FORCE();
    arma::vec get_TORQUE();
    arma::mat get_M();
    arma::vec get_TBI_Q();
    arma::vec get_TBID_Q();
    unsigned int get_num();

    void set_POSITION(const arma::vec PosIn);
    void set_VELOCITY(const arma::vec VelIn);
    void set_ACCELERATION(const arma::vec AccIn);
    void set_ANGLE(const arma::vec AngIn);
    void set_ANGLE_VEL(const arma::vec AngvelIn);
    void set_ANGLE_ACC(const arma::vec AngaccIn);

    virtual void update(double MassIn, arma::mat IIn, arma::vec PosIn, arma::vec VelIn, arma::vec TBI_QIn, arma::vec ANG_VEL_In, arma::vec FIn, arma::vec TIn) = 0;

protected:
    unsigned int type;  // type define   0: Ground body, 1: Mobilized body
    unsigned int num;   // No. body

    VECTOR(POSITION, 3);
    VECTOR(VELOCITY, 3);
    VECTOR(ACCELERATION, 3);
    VECTOR(ANGLE, 3);
    VECTOR(ANGLE_VEL, 3);
    VECTOR(ANGLE_ACC, 3);
    MATRIX(M, 6, 6);
    VECTOR(FORCE, 3);
    VECTOR(TORQUE, 3);
    VECTOR(APPILED_TORQUE, 3);
    MATRIX(TBI, 3, 3);
    VECTOR(TBI_Q, 4);
    VECTOR(TBID_Q, 4);
};

class Ground : public Body
{
public:
    Ground(unsigned int NumIn);
    ~Ground(){};
    virtual void update(double MassIn, arma::mat IIn, arma::vec PosIn, arma::vec VelIn, arma::vec TBI_QIn, arma::vec ANG_VEL_In, arma::vec FIn, arma::vec TIn){};
};

class Mobilized_body : public Body
{
public:
    Mobilized_body(unsigned int NumIn, arma::vec PosIn, arma::vec VelIn, arma::vec AccIn, arma::vec TBI_QIn, arma::vec ANG_VEL_In, arma::vec ANG_ACC_In, double MIn, arma::mat IIn);
    ~Mobilized_body(){};

    virtual void update(double MassIn, arma::mat IIn, arma::vec PosIn, arma::vec VelIn, arma::vec TBI_QIn, arma::vec ANG_VEL_In, arma::vec FIn, arma::vec TIn) override;
};

#endif  //BODY_HPP