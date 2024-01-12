#ifndef __DFA_H
#define __DFA_H

#include <string>
typedef enum {
#define X(a) a,
#include "states.def"
#undef X
} state_t;
void move_to_next_state();
std::string state_name(state_t);
#endif

