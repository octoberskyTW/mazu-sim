#ifndef BODY_HPP
#define BODY_HPP

#include <armadillo>
#include <memory>
#include "matrix_tool.hh"
#include "aux.hh"

class Body
{
public:
    Body();
    virtual ~Body() = default;

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

    void set_POSITION(const arma::vec &PosIn);
    void set_VELOCITY(const arma::vec &VelIn);
    void set_ACCELERATION(const arma::vec &AccIn);
    void set_ANGLE(const arma::vec &AngIn);
    void set_ANGLE_VEL(const arma::vec &AngvelIn);
    void set_ANGLE_ACC(const arma::vec &AngaccIn);
    void set_TBI(const arma::mat &TBIIn);
    void set_FORCE(const arma::vec &ForceIn);

    virtual void update(arma::vec PosIn, arma::vec VelIn, arma::vec AttIn
        , arma::vec ANG_VEL_In) = 0;

protected:

    unsigned int type;  // type define   0: Ground body, 1: Mobilized body
    unsigned int num;  // No. body

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
    ~Ground() {};
    virtual void update(arma::vec , arma::vec , arma::vec 
        , arma::vec ) {};
};

class Mobilized_body : public Body
{
    TRICK_INTERFACE(Mobilized_body);
public:
    Mobilized_body(unsigned int NumIn, const arma::vec PosIn, const arma::vec VelIn, const arma::vec AccIn, const arma::mat TBIIn
        , const arma::vec ANG_VEL_In, const arma::vec ANG_ACC_In, double MIn, const arma::mat IIn
        , const arma::vec F_In, const arma::vec T_In);
    Mobilized_body(unsigned int NumIn, const arma::vec PosIn, const arma::vec VelIn, const arma::vec AccIn, const arma::mat TBIIn
        , const arma::vec ANG_VEL_In, const arma::vec ANG_ACC_In, double MIn, const arma::mat IIn
        , const arma::vec F_In);
    ~Mobilized_body() {};

    virtual void update(arma::vec PosIn, arma::vec VelIn, arma::vec TBI_QIn
        , arma::vec ANG_VEL_In) override;
};
#endif  //BODY_HPP