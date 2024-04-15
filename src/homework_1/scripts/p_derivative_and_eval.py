from sympy import symbols, sin, cos, diff

g, m1, m2, L1, L2, X1, X2, X3, X4 = symbols('g m1 m2 L1 L2 X1 X2 X3 X4')

diff_var = X1

eval_var = {
    X1 : 0,
    X2 : 0,
    X3 : 0,
    X4 : 0
}
#X3_dot
# equation = (-g*(2*m1+m2)*sin(X1) - m2*g*sin(X1 - 2*X2) - 2*sin(X1-X2)*m2*(X4**2*L2 + X3**2*L1*cos(X1 - X2))) / \
#            (L1*(2*m1+m2-m2*cos(2*X1 - 2*X2)))
#X4_dot
equation = (2*sin(X1-X2)*(X3**2*L1*(m1+m2) + g*(m1+m2)*cos(X1) + X4**2*L2*m2*cos(X1-X2))) / \
           (L2*(2*m1+m2-m2*cos(2*X1-2*X2)))

partial_derivative = diff(equation, X4)

evaluated = partial_derivative.subs(eval_var)

print("X4 = " + str(evaluated))