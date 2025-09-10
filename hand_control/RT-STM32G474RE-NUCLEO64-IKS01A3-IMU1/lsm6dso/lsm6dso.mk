# lsm6dso PATH
LSM6DOPATH = ./lsm6dso

# List of all the LSM6DO device files.
LSM6DOSRC := $(LSM6DOPATH)/lsm6dso.c

# Required include directories
LSM6DOINC := $(LSM6DOPATH)

# Shared variables
ALLCSRC += $(LSM6DOSRC)
ALLINC  += $(LSM6DOINC)