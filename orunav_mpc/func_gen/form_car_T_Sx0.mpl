# Car-like vehicle: form T and S*x0 in X = S*x0 + T*U
#=====================================================================

# -------------------------------------------------------------------
# Loading libraries
# -------------------------------------------------------------------
with(LinearAlgebra):
with(codegen, C, makeproc):
# -------------------------------------------------------------------
Nx := 4:            # state dimension
Nu := 2:            # control dimension

theta := vector(N): # vehicle orientation
phi := vector(N):   # steering angle
v := vector(N):     # linear velocity

# initial state
xx := vector(Nx):
x0 := Vector(Nx,i->xx[i]):

# -------------------------------------------------------------------

A := (dt,theta,phi,v,l) -> Matrix([[1,  0, -sin(theta)*v*dt,                    0],
                                   [0,  1,  cos(theta)*v*dt,                    0],
                                   [0,  0,                1,  v*dt/(l*cos(phi)^2)],
                                   [0,  0,                0,                    1]]):

B := (dt,theta,phi,l) -> Matrix([[cos(theta)*dt,  0],
                                 [sin(theta)*dt,  0],
                                 [dt*tan(phi)/l,  0],
                                 [            0, dt]]):

# -------------------------------------------------------------------

# form S
S := Matrix(Nx*N,Nx,0):
for i from 1 to N do
    ind_x := Nx*(i-1)+1..Nx*i;
    S[ind_x,1..Nx] := IdentityMatrix(Nx);
    for k from 1 to i do
        S[ind_x,1..Nx] := A(dt,theta[k],phi[k],v[k],l).S[ind_x,1..Nx];
    end do:
end do:

# form T (this can be implemented much more intelligently)
T := Matrix(Nx*N,Nu*N,0):
for i from 1 to N do
    ind_u := Nu*(i-1)+1..Nu*i;
    for j from i to N do
        ind_x := Nx*(j-1)+1..Nx*j;
        T[ind_x,ind_u] := B(dt,theta[i],phi[i],l);
        for k from i+1 to j do
            T[ind_x,ind_u] := A(dt,theta[k],phi[k],v[k],l).T[ind_x,ind_u];
        end do:
    end do:
end do:

Sx0 := S.x0:

# -------------------------------------------------------------------
# Code generation
# -------------------------------------------------------------------

out := ArrayTools:-Concatenate(2,convert(Transpose(T),vector),convert(Sx0,vector)):

alias(fname = form_car_T_Sx0):

printf("makeproc ...\n");
st := time():

fname := makeproc(out,[dt, l, theta::array(1..N), phi::array(1..N), v::array(1..N), xx::array(1..Nx)]):

printf("completed in %f seconds\n\n",time()-st);

printf("Generating %s ...\n",cat(fname,"_",N,".c"));
st := time():

fd := fopen(cat("./",cat(fname,"_",N,".c")),WRITE):
fprintf(fd,"/* Generated using codegen (%s) */\n", StringTools:-FormatTime("%Y-%m-%d, %T")):
fclose(fd):

C(fname, optimized, filename = cat("./",cat(fname,"_",N,".c")));

printf("completed in %f seconds\n\n",time()-st);
