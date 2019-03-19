# Makefile for optitrack-gazebo
#
#                                      Anthony Mallet on Tue Mar 19 2019
#
ACLOCAL_AMFLAGS=	-I autoconf

# pkg-config
pkgconfigdir=		$(libdir)/pkgconfig
pkgconfig_DATA=		optitrack-gazebo.pc

# plugin
plugindir=		$(libdir)/gazebo
plugin_LTLIBRARIES =	optitrack-gazebo.la

optitrack_gazebo_la_SOURCES=\
	src/plugin.cc	\
	src/natnet.h	\
	src/natnet.cc

optitrack_gazebo_la_CPPFLAGS=	$(dependencies_CFLAGS) $(BOOST_CPPFLAGS)
optitrack_gazebo_la_LDFLAGS=	-module -release ${PACKAGE_VERSION}
optitrack_gazebo_la_LIBADD=	$(dependencies_LIBS)

# doc
dist_doc_DATA=	README.html README.adoc

%.html: %.adoc
	$(ASCIIDOCTOR) $(ASCIIDOCTOR_ARGS) -b html5 -o $@ $<
