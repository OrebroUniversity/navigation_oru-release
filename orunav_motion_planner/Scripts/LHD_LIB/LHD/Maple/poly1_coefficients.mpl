#
# Find the coefficients of a third order polynomial given 4 conditions
#

#--------------------------------------------------------------------------------
# Loading libraries
#--------------------------------------------------------------------------------
with(LinearAlgebra):
with(codegen, C, makeproc):
#--------------------------------------------------------------------------------

A := Matrix([[ 1, t_i],
             [ 1, t_f]]):

b := < p_i; p_f>:

iA := MatrixInverse(A):
p := iA.b:

all := convert(p,vector):

poly1_coefficients := makeproc(all,[t_i, t_f, p_i, p_f]):

fd := fopen("poly1_coefficients.c",WRITE):
fprintf(fd,"/* drdv: generated using codegen (%s) */\n", StringTools:-FormatTime("%Y-%m-%d, %T")):
fclose(fd):

C(poly1_coefficients, optimized, filename = "poly1_coefficients.c"):
