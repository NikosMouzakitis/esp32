#pragma once
#include <cstdarg>
namespace Eloquent {
    namespace ML {
        namespace Port {
            class PWMSVM {
                public:
                    /**
                    * Predict class for features vector
                    */
                    int predict(float *x) {
                        float kernels[8] = { 0 };
                        float decisions[3] = { 0 };
                        int votes[3] = { 0 };
                        kernels[0] = compute_kernel(x,   0.608167089637  , 0.470903717396  , 0.472690321775  , 0.5012207797  , 0.445772849188  , 0.44470449809  , 1.282500567779  , 1.251548650714  , 1.235502987513 );
                        kernels[1] = compute_kernel(x,   0.1201549202  , 0.083726443355  , 0.751500687591  , 0.740127946835  , 0.6737640042  , 0.666348326711  , 0.651389014156  , 0.927395347978  , 0.924794718272 );
                        kernels[2] = compute_kernel(x,   -0.156874544405  , 1.181213904358  , 1.085570765551  , 1.026386084033  , 0.946942595341  , 0.931921562806  , 0.903643970225  , 0.886762046649  , 0.885846780297 );
                        kernels[3] = compute_kernel(x,   0.28251987131  , 0.212542942425  , 0.249767619377  , 0.310202661832  , 0.263482623897  , 0.267489274756  , 0.272532416602  , 0.279088742505  , -1.249725549134 );
                        kernels[4] = compute_kernel(x,   -2.500617200251  , -1.995531962537  , -1.655436547034  , -1.322329646926  , -1.249269499225  , -1.203147480009  , -1.124353110801  , -1.065919186877  , -0.985842328919 );
                        kernels[5] = compute_kernel(x,   0.233902117588  , 0.173970770387  , 0.406127486693  , 0.444184383942  , 0.391342528397  , 0.391789529996  , 0.390599115965  , 0.392770675437  , 0.396590660563 );
                        kernels[6] = compute_kernel(x,   -0.319239495514  , -0.264878658648  , -0.162168416604  , -0.042777296818  , -0.073369172698  , -0.007069521958  , 0.011742518411  , 0.027984071371  , 0.062688675449 );
                        kernels[7] = compute_kernel(x,   0.62467877958  , 0.484003700352  , 0.528577985193  , 0.549109828968  , 0.491473778909  , 0.489133103377  , 0.48306098896  , 0.481798695202  , 0.45085520336 );
                        decisions[0] = -0.333320714743
                        + kernels[1] * 0.327905429328
                        + kernels[3] * -0.327905429328
                        ;
                        decisions[1] = -2.301164888506
                        + kernels[0] * 0.518488915201
                        + kernels[1]
                        + kernels[2] * 0.219525862482
                        + kernels[5] * -0.096000954988
                        + kernels[7] * -1.642013822695
                        ;
                        decisions[2] = -0.91312917889
                        + kernels[3] * 0.983829034846
                        + kernels[4] * 0.281593108866
                        + kernels[6] * -1.265422143712
                        ;
                        votes[decisions[0] > 0 ? 0 : 1] += 1;
                        votes[decisions[1] > 0 ? 0 : 2] += 1;
                        votes[decisions[2] > 0 ? 1 : 2] += 1;
                        int val = votes[0];
                        int idx = 0;

                        for (int i = 1; i < 3; i++) {
                            if (votes[i] > val) {
                                val = votes[i];
                                idx = i;
                            }
                        }

                        return idx;
                    }

                    /**
                    * Predict readable class name
                    */
                    const char* predictLabel(float *x) {
                        return idxToLabel(predict(x));
                    }

                    /**
                    * Convert class idx to readable name
                    */
                    const char* idxToLabel(uint8_t classIdx) {
                        switch (classIdx) {
                            case 0:
                            return "INCREASE";
                            case 1:
                            return "DECREASE";
                            case 2:
                            return "NEUTRAL";
                            default:
                            return "Houston we have a problem";
                        }
                    }

                protected:
                    /**
                    * Compute kernel between feature vector and support vector.
                    * Kernel type: linear
                    */
                    float compute_kernel(float *x, ...) {
                        va_list w;
                        va_start(w, 9);
                        float kernel = 0.0;

                        for (uint16_t i = 0; i < 9; i++) {
                            kernel += x[i] * va_arg(w, double);
                        }

                        return kernel;
                    }
                };
            }
        }
    }
