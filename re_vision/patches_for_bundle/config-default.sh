#!/bin/bash
#
# 3D Model Creator
# config.sh
# 2010 Dorian Galvez-Lopez. University of Zaragoza
#
#
# Configuration parameters for installing and creating 3D models
#


# Config from here -----------------------------------------------------------

# Path to the root directory of the bundler source distribution
export BUNDLER_BASE="/opt/bundler-v0.4-source"

# Path to the PMVS root directory
export PMVS_BASE="/opt/pmvs-2"

# End of configuration -------------------------------------------------------

export BUNDLER=$BUNDLER_BASE/bin/bundler
export BUNDLE2PMVS=$BUNDLER_BASE/bin/Bundle2PMVS
export BUNDLER_SRC=$BUNDLER_BASE/src
export PMVS=$PMVS_BASE/program/main/pmvs2


