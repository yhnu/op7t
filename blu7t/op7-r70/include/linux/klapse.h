#ifndef _KLAPSE_H
#define _KLAPSE_H

/* KLAPSE_MDSS : Use 1 if using with MDSS */
#define KLAPSE_MDSS 0

/* set_rgb_slider : Interface function for brightness-mode */
typedef u32 bl_type_t;
extern void set_rgb_slider(bl_type_t bl_lvl);

/* Variable type for rgb */
typedef unsigned short col_type_t;

#if KLAPSE_MDSS
 #define K_RED    kcal_get_color(0)
 #define K_GREEN  kcal_get_color(1)
 #define K_BLUE   kcal_get_color(2)

 extern col_type_t kcal_get_color(unsigned short int code);
 extern void klapse_kcal_push(int r, int g, int b);
#else
 #define K_RED    kcal_red
 #define K_GREEN  kcal_green
 #define K_BLUE   kcal_blue

 extern col_type_t K_RED, K_GREEN, K_BLUE;
#endif

/* Constants - Customize as needed */
#define DEFAULT_ENABLE 0 /* 0 = off, 1 = time-based, 2 = brightness-based */

#define MAX_SCALE 256 /* Maximum value of RGB possible */

#define MIN_SCALE 20 /* Minimum value of RGB recommended */

#define MAX_BRIGHTNESS 1023 /* Maximum display brightness */

#define MIN_BRIGHTNESS 2 /* Minimum display brightness */

#define UPPER_BL_LVL 400 /* Upper target for brightness-dependent mode */

#define LOWER_BL_LVL 2 /* Lower target for brightness-dependent mode */

#define DEFAULT_FLOW_FREQ 360 /* Flow delays for rapid pushes in mode 2 */

#endif  /* _KLAPSE_H */

