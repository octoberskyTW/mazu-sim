#ifndef BODY_HPP
#define BODY_HPP

#include <armadillo>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include "matrix_tool.hh"

class Body : public boost::enable_shared_from_this<Body>
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

    virtual void update(arma::vec PosIn, arma::vec VelIn, arma::vec AttIn
        , arma::vec ANG_VEL_In) = 0;

protected:

    unsigned int type;  // type define   0: Ground body, 1: Mobilized body
    unsigned int num;  // No. body

    arma::vec POSITION;
    arma::vec VELOCITY;
    arma::vec ACCELERATION;
    arma::vec ANGLE;
    arma::vec ANGLE_VEL;
    arma::vec ANGLE_ACC;
    arma::mat M;
    arma::vec FORCE;
    arma::vec TORQUE;
    arma::vec APPILED_TORQUE;
    arma::mat TBI;
    arma::vec TBI_Q;
    arma::vec TBID_Q;
};

class Ground : public Body
{
public:
    Ground(unsigned int NumIn);
    ~Ground() {};
    virtual void update([[maybe_unused]]arma::vec PosIn, [[maybe_unused]]arma::vec VelIn, [[maybe_unused]]arma::vec AttIn
        , [[maybe_unused]]arma::vec ANG_VEL_In) {};
};

class Mobilized_body : public Body
{
public:
    Mobilized_body(unsigned int NumIn, arma::vec PosIn, arma::vec VelIn, arma::vec AccIn, arma::vec AttIn
        , arma::vec ANG_VEL_In, arma::vec ANG_ACC_In, double MIn, arma::vec IIn
        , arma::vec F_In, arma::vec T_In);
    ~Mobilized_body() {};

    virtual void update(arma::vec PosIn, arma::vec VelIn, arma::vec TBI_QIn
        , arma::vec ANG_VEL_In) override;
};

typedef boost::shared_ptr<Body> BodyPtr;
#endif  //BODY_HPP