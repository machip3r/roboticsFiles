"""
    // 16/Feb/2023

    Marcos de referencia variantes en el tiempo

    Dado que para SO(3) R ^ -1 = R ^ T,
    si R depende del tiempo entonces
        R(t)(R(t) ^ T) = I

    R_z(t) = | cos t  -sin t  0 |
             | sin t   cos t  0 |
             |     0       0  1 |

    Derivando con respecto a t
        R'(t)(R(t) ^ T) + R(t)(R'(t) ^ T) = 0
        R'(t)(R(t) ^ T) = -R(t)(R'(t) ^ T)

        (AB) ^ T = (B ^ T)(A ^ T)

        R'(t)R(t) = -(R'(t)(R(t) ^ T) ^ T)

    Existe algún vector w(t) tal que
        R'(t)(R(t) ^ T) = S(w(t))
        A   A   A
        |   |   |
          matriz
        antisimétrica
           A   A   A
           |   |   |
        |   0  -Vx   Vy |   | Vx |
        |  Vx    0  -Vz |  S| Vy |
        | -Vy   Vz    0 |   | Vz |

        R'(t) = S(w(t))R(t)

    Aproximando la derivada
        R'(t) =(aprox)= R(t + d_t) - R(t)
                        ----------------
                               d_t
        con d_t =(aprox)= 0

        R(t + d_t) = d_t * R'(t) + R(t)
        R(t + d_t) = d_t * S(w(t))R'(t) + R(t)
                  = (d_t * S(w(t)) + I) * R(t)

    Movimiento incremental

    Una rotación pequeña de R_0 a R_1

        R_1 = (d_t * S(w) + I) * R_0
        d_t * S(w) = R_1 * R_0 ^ T - I

    Denotando como vex(·) a la operación inversa a S(·)

    d_theta = vex(R_1 * R_0 ^ T - I)
    d_theta = d_t * w

    d_theta -> Rotación infinitecimal con respecto a los ejes x, y, z globales


    Dadas dos poses E_0 y E_1 que difieren infinitecimalmente, su diferencia es

        d = increment(T_0, T_1) = | t_1 - t_0             | = | d_d     |
                                  | vex(R_1, R_0 ^ T - I) |   | d_theta |

    d -> velocidad espacial multiplicada por S_t

        T = | S(d_theta) d_d | + I_4x4
            | 0_3x1        0 |

    // 20/Feb/2023

    Modelado de robots móviles

    Robot (Modelo) <---> E.D.O.

        x'(t) = x(t + increment(t)) - x(t)
                --------------------------
                      increment(t)
        Aprox
            x(t + increment(t)) = x(t) + increment(t) + x'(t)

"""

"""
    Mapas de rejilla de ocupación

    Matriz donde las celdas se marcan como obstáculos

      0 - libre
      1 - obstáculo
    0.5 ó -1 - sin explorar


"""
