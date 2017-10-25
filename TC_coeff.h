// Coefficients for inverse polinomial to compute temp from voltage. 
#define N_poly 9	//  see table below
// K-Type coeff

// this table should only be used up to 20mv , 0 to 500C
float K_TC_coeff0_500[]= {            
			0.000000E+00,
            2.508355E+01,
            7.860106E-02,
            -2.503131E-01,
            8.315270E-02,
            -1.228034E-02,
            9.804036E-04,
            -4.413030E-05,
            1.057734E-06,
            -1.052755E-08
};

float K_TC_coeff500_plus [] = {
            -1.318058E+02,
            4.830222E+01,
            -1.646031E+00,
            5.464731E-02,
            -9.650715E-04,
            8.802193E-06,
            -3.110810E-08,
            0.000000E+00,
            0.000000E+00,
            0.000000E+00,
};