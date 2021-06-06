#include "const.h"

#if WORLD == 0
const position_t MIGR[1];
const position_t INIT_POS[1] = {
    {-2.9, 0., -M_PI / 2}
};
const float FORM_REL_POS[5][2];
#elif WORLD == 1
// const position_t MIGR[] = {{2., 0.5, 0}};
// const position_t MIGR[] = {{2., -0.5, 0}};
// const position_t MIGR[] = {{2., 0, 0}};
// const position_t MIGR[] = {{1.5, 1.2, 0}};
const position_t MIGR[] = {{1.5, -1.2, 0}};
const position_t INIT_POS[] = {
    {-2.9, 0., -M_PI / 2},
    {-2.9, -0.1, -M_PI / 2},
    {-2.9, 0.1, -M_PI / 2},
    {-2.9, -0.2, -M_PI / 2},
    {-2.9, 0.2, -M_PI / 2},
};
const float FORM_REL_POS[5][2] = {
    {0., 0.},
    {-0.2, -0.2},
    {-0.2, 0.2},
    {0.2, -0.2},
    {0.2, 0.2}
};
#elif WORLD == 2
const position_t MIGR[] = {{-1.7, 0, 0}, {-0.3, 0, 0}};
// const position_t MIGR[] = {{-1.7, 0.2, 0}, {-0.3, 0.2, 0}};
// const position_t MIGR[] = {{-1.7, 0.2, 0}, {-0.3, -0.2, 0}};
// const position_t MIGR[] = {{-1.6, -0.1, 0}, {-0.4, .1, 0}};
// const position_t MIGR[] = {{-1.6, 0.1, 0}, {-0.4, -.1, 0}};
const position_t INIT_POS[] = {
    {-0.1, 0, M_PI / 2},
    {-0.1, 0.1, M_PI / 2},
    {-0.1, -0.1, M_PI / 2},
    {-0.1, 0.2, M_PI / 2},
    {-0.1, -0.2, M_PI / 2},
    {-1.9, 0, -M_PI / 2},
    {-1.9, 0.1, -M_PI / 2},
    {-1.9, -0.1, -M_PI / 2},
    {-1.9, 0.2, -M_PI / 2},
    {-1.9, -0.2, -M_PI / 2},
};
const float FORM_REL_POS[5][2] = {
    {0., 0.},
    {-0.15, 0.2},
    {-0.15, -0.2},
    {0.15, 0.15},
    {0.15, -0.15}
};
#elif WORLD == 4
const position_t MIGR[] = {{-2.5, 0, 0}, {-0.5, 0, 0}};
// const position_t MIGR[] = {{-2.5, 0.2, 0}, {-0.5, 0.2, 0}};
// const position_t MIGR[] = {{-2.5, 0.2, 0}, {-0.5, -0.2, 0}};
// const position_t MIGR[] = {{-2.6, -0.1, 0}, {-0.4, .1, 0}};
// const position_t MIGR[] = {{-2.6, 0.1, 0}, {-0.4, -.1, 0}};
const position_t INIT_POS[] = {
    {-0.1, 0, M_PI / 2},
    {-0.1, 0.1, M_PI / 2},
    {-0.1, -0.1, M_PI / 2},
    {-0.1, 0.2, M_PI / 2},
    {-0.1, -0.2, M_PI / 2},
    {-2.9, 0, -M_PI / 2},
    {-2.9, 0.1, -M_PI / 2},
    {-2.9, -0.1, -M_PI / 2},
    {-2.9, 0.2, -M_PI / 2},
    {-2.9, -0.2, -M_PI / 2},
};
const float FORM_REL_POS[5][2] = {
    {0., 0.},
    {-0.15, 0.2},
    {-0.15, -0.2},
    {0.15, 0.15},
    {0.15, -0.15}
};
#endif
