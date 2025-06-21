<<<<<<< HEAD
#include "jtest.h"
#include "matrix_tests.h"

JTEST_DEFINE_GROUP(matrix_tests)
{
    /*
      To skip a test, comment it out.
    */
    JTEST_GROUP_CALL(mat_add_tests);
    JTEST_GROUP_CALL(mat_cmplx_mult_tests);
    JTEST_GROUP_CALL(mat_init_tests);
    JTEST_GROUP_CALL(mat_inverse_tests);
    JTEST_GROUP_CALL(mat_mult_tests);
    JTEST_GROUP_CALL(mat_mult_fast_tests);
    JTEST_GROUP_CALL(mat_sub_tests);
    JTEST_GROUP_CALL(mat_trans_tests);
    JTEST_GROUP_CALL(mat_scale_tests);
    return;
}
=======
#include "jtest.h"
#include "matrix_tests.h"

JTEST_DEFINE_GROUP(matrix_tests)
{
    /*
      To skip a test, comment it out.
    */
    JTEST_GROUP_CALL(mat_add_tests);
    JTEST_GROUP_CALL(mat_cmplx_mult_tests);
    JTEST_GROUP_CALL(mat_init_tests);
    JTEST_GROUP_CALL(mat_inverse_tests);
    JTEST_GROUP_CALL(mat_mult_tests);
    JTEST_GROUP_CALL(mat_mult_fast_tests);
    JTEST_GROUP_CALL(mat_sub_tests);
    JTEST_GROUP_CALL(mat_trans_tests);
    JTEST_GROUP_CALL(mat_scale_tests);
    return;
}
>>>>>>> b334a05 (Week 3 Tasks)
