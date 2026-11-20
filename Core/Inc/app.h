#ifndef APP_H
#define APP_H

#include "types.h"

extern Diag     g_diag;
extern AppState g_app;

void app_init(void);
void app_loop(void);

#endif
