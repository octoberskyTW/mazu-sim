#include "Joint.hh"

Joint::Joint(unsigned int TypeIn, arma::vec piIn, arma::vec pjIn, arma::vec qiIn,
            arma::vec qjIn, Body *i_In, Body *j_In) :
    VECTOR_INIT(pi, 3),
    VECTOR_INIT(pj, 3),
    VECTOR_INIT(qi, 3),
    VECTOR_INIT(qj, 3),
    MATRIX_INIT(Cqi, 3, 6),
    MATRIX_INIT(Cqj, 3, 6),
    VECTOR_INIT(GAMMA, 3),
    VECTOR_INIT(CONSTRAINT, 3),
    MATRIX_INIT(TBI_i, 3, 3),
    MATRIX_INIT(TBI_j, 3, 3),
    VECTOR_INIT(Pi, 3),
    VECTOR_INIT(Pj, 3),
    VECTOR_INIT(Qi, 3),
    VECTOR_INIT(Qj, 3),
    VECTOR_INIT(wi, 3),
    VECTOR_INIT(wj, 3),
    VECTOR_INIT(Si, 3),
    VECTOR_INIT(Sj, 3) {
        Type = TypeIn;
        body_i_ptr = i_In;
        body_j_ptr = j_In;
        pi = piIn;
        pj = pjIn;
        qi = qiIn;
        qj = qjIn;

        update();
}

void Joint::update() {
    TBI_i = body_i_ptr->get_TBI();
    TBI_j = body_j_ptr->get_TBI();
    wi = body_i_ptr->get_ANGLE_VEL();
    wj = body_j_ptr->get_ANGLE_VEL();
    Pi = trans(TBI_i) * pi;
    Pj = trans(TBI_j) * pj;
    Qi = trans(TBI_i) * qi;
    Qj = trans(TBI_j) * qj;
    Si = body_i_ptr->get_POSITION();
    Sj = body_j_ptr->get_POSITION();

    Build_C();
    Build_Cq();
    Build_GAMMA();
}

void Joint::Build_C() {
    CONSTRAINT = Si + Pi - Sj - Pj;
}

void Joint::Build_Cq() {
    arma::mat temp_matrix_1(3, 3, arma::fill::eye);
    arma::mat temp_matrix_2(3, 3, arma::fill::zeros);
    arma::mat temp_matrix_3(3, 3, arma::fill::eye);
    arma::mat temp_matrix_4(3, 3, arma::fill::zeros);

    temp_matrix_2 = -skew_sym(Pi) * trans(TBI_i);
    temp_matrix_4 = skew_sym(Pj) * trans(TBI_j);

    Cqi = join_rows(temp_matrix_1, temp_matrix_2);
    Cqj = join_rows(-temp_matrix_3, temp_matrix_4);
}

void Joint::Build_GAMMA() {
    arma::mat33 Skew_Omega_i = skew_sym(wi);
    arma::mat33 Skew_Omega_j = skew_sym(wj);

    GAMMA = -trans(TBI_i) * Skew_Omega_i * Skew_Omega_i * pi + trans(TBI_j) * Skew_Omega_j * Skew_Omega_j * pj;
}

arma::mat Joint::get_Cqi() { return Cqi; }
arma::mat Joint::get_Cqj() { return Cqj; }
arma::vec Joint::get_CONSTRAINT() { return CONSTRAINT; }
arma::vec Joint::get_GAMMA() { return GAMMA; }
arma::vec Joint::get_Pi() { return Pi; }
arma::vec Joint::get_Pj() { return Pj; }
arma::vec Joint::get_pi() { return pi; }
arma::vec Joint::get_pj() { return pj; }
Body* Joint::get_body_i_ptr() { return body_i_ptr; }
Body* Joint::get_body_j_ptr() { return body_j_ptr; }









