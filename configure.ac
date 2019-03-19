dnl Autoconf file for optitrack-gazebo
dnl
dnl                                      Anthony Mallet on Tue Mar 19 2019
dnl
AC_PREREQ(2.59)

AC_INIT([optitrack-gazebo],[1.0],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acoptitrack.h])
AM_INIT_AUTOMAKE([foreign no-define subdir-objects])


dnl Compilers
dnl
LT_INIT([disable-static])
AC_PROG_CXX


dnl External packages
dnl
AX_BOOST_BASE([1.40],, [AC_MSG_ERROR([boost not available])])
PKG_CHECK_MODULES(dependencies, [gazebo])


dnl Doc
AM_MISSING_PROG([ASCIIDOCTOR], [asciidoctor])


dnl Output
AC_CONFIG_FILES([
	optitrack-gazebo.pc
	Makefile
])
AC_OUTPUT
