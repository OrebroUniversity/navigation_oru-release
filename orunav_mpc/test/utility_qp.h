#include "utility.h"


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// reference trajectory
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <iostream>
#include <vector>
#include <fstream>
#include <cstdio>



enum refTrajReturn
{
    REF_TRAJECTORY_RETURN_OK,
    REF_TRAJECTORY_RETURN_STOP
};


/******************************************/
/** \brief Defines one reference trajectory for state and control */
class referenceTrajectory
{
    public:
        unsigned int start_pos;

        /** \brief Reference state trajectory
            \note states[0] is assumed to be the state that is reached by applying controls[0] from x0=(0,0,0,0)
         */
        vector<State> states;

        /** \brief Reference control trajectory */
        vector<Control> controls;


        State init_state;

        double x[SW_PREVIEW_WINDOW_LEN];
        double y[SW_PREVIEW_WINDOW_LEN];
        double theta[SW_PREVIEW_WINDOW_LEN];
        double phi[SW_PREVIEW_WINDOW_LEN];

        double v[SW_PREVIEW_WINDOW_LEN];
        double w[SW_PREVIEW_WINDOW_LEN];


        referenceTrajectory(const char *filename)
        {
            start_pos = 0;
            ifstream ifs;

            ifs.open(filename);
            if(!ifs)
            {
                cout << "Cannot open trajectory file." << endl;
                exit(1);
            }

            while (!ifs.eof())
            {
                State state;
                Control control;
                string line;

                getline(ifs, line);
                if (sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf",
                        &state.x(), &state.y(), &state.theta(), &state.phi(),
                        &control.v(), &control.w()) == 6)
                {
                    states.push_back(state);
                    controls.push_back(control);
                }
            }

            ifs.close();
        }


        refTrajReturn formPreview()
        {
            if (start_pos + SW_PREVIEW_WINDOW_LEN + 1 > controls.size())
            {
                return REF_TRAJECTORY_RETURN_STOP;
            }
            init_state = states[start_pos];
            for (unsigned int i = start_pos + 1, k = 0; k < SW_PREVIEW_WINDOW_LEN; ++i, ++k)
            {
                x[k]     = states[i].x();
                y[k]     = states[i].y();
                theta[k] = states[i].theta();
                phi[k]   = states[i].phi();

                v[k] = controls[i].v();
                w[k] = controls[i].w();
            }
            ++start_pos;
            return REF_TRAJECTORY_RETURN_OK;
        }
};
