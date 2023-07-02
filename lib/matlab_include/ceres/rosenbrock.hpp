// Copyright 2021-2022 The MathWorks, Inc.
// Exercising Ceres's general unconstrained minimization interface
#ifdef BUILDING_LIBMWCERESCODEGEN
    #include "cerescodegen/cerescodegen_spec.hpp"
#else
    /* To deal with the fact that PackNGo has no include file hierarchy */
    #include "cerescodegen_spec.hpp"
#endif

#include "ceres/ceres.h"

#ifndef ROSENBROCK_HPP
#define ROSENBROCK_HPP

// f(x,y) = (1-x)^2 + 100(y - x^2)^2;
namespace mw_ceres { 
    class CERESCODEGEN_API Rosenbrock : public ceres::FirstOrderFunction {
    public:
        virtual ~Rosenbrock() override {}

        virtual bool Evaluate(const double* parameters,
                                    double* cost,
                                    double* gradient) const override{
            const double x = parameters[0];
            const double y = parameters[1];

            cost[0] = (1.0 - x) * (1.0 - x) + 100.0 * (y - x * x) * (y - x * x);
            // analytical Jacobian
            if (gradient != NULL) {
                gradient[0] = -2.0 * (1.0 - x) - 200.0 * (y - x * x) * 2.0 * x;
                gradient[1] = 200.0 * (y - x * x);
            }
            return true;
        }

        virtual int NumParameters() const override { return 2; }
    };
}  // namespace mw_ceres
#endif // ROSENBROCK_HPP


