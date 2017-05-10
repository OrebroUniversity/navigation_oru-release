# Vehicle tracking controller
#=====================================================================

# -------------------------------------------------------------------
# Loading libraries
# -------------------------------------------------------------------
with(LinearAlgebra):
with(codegen, C, makeproc):
# -------------------------------------------------------------------
Nx := 4:   # state dimension
Nu := 2:   # control dimension

theta := vector(N): # vehicle orientation
phi := vector(N):   # steering angle
v := vector(N):     # linear velocity

# initial state
xx := vector(Nx):
x0 := Vector(Nx,i->xx[i]):

# -------------------------------------------------------------------
# Gains: Gain_state, Gain_control

# Penalty matrices
Qk := IdentityMatrix(Nx)*Gain_state:
Rk := IdentityMatrix(Nu)*Gain_control:
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
Q := Matrix(Nx*N,Nx*N,0):
for i from 1 to N do
    Q[Nx*i-(Nx-1)..Nx*i,Nx*i-(Nx-1)..Nx*i] := Qk:
end do:
# -------------------------------------------------------------------
R := Matrix(Nu*N,Nu*N,0):
for i from 1 to N do
    R[Nu*i-(Nu-1)..Nu*i,Nu*i-(Nu-1)..Nu*i] := Rk:
end do:
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

# form T
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

P := Transpose(T).Q.T+R:
p := 2*Transpose(T).Q.S.x0:

Sx0 := S.x0:

# -------------------------------------------------------------------
# Code generation
# -------------------------------------------------------------------

out := ArrayTools:-Concatenate(2,convert(P,vector),convert(p,vector),convert(Transpose(S),vector),convert(Transpose(T),vector),convert(Sx0,vector)):

#out := ArrayTools:-Concatenate(2,convert(Transpose(S),vector),convert(Transpose(T),vector)):

alias(fname = form_car_data):

printf("makeproc ...\n");
st := time():

fname := makeproc(out,[Gain_state, Gain_control, dt, theta::array(1..N),phi::array(1..N),v::array(1..N),l,xx::array(1..Nx)]):

printf("completed in %f seconds\n\n",time()-st);

printf("Generating %s ...\n",cat(fname,".c"));
st := time():

fd := fopen(cat("./",cat(fname,".c")),WRITE):
fprintf(fd,"/* Generated using codegen (%s) */\n", StringTools:-FormatTime("%Y-%m-%d, %T")):
fclose(fd):

C(fname, optimized, filename = cat("./",cat(fname,".c")));

printf("completed in %f seconds\n\n",time()-st);

