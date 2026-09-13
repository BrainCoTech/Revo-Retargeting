/* Evaluate full FBX transforms with pinned ufbx, including pivots and pre/post rotations. */
#include "ufbx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int cmp_time(const void *a, const void *b) {
    double x = *(const double*)a, y = *(const double*)b;
    return (x > y) - (x < y);
}
static void string_json(const char *s) {
    putchar('"');
    for (; *s; ++s) {
        unsigned char c = (unsigned char)*s;
        if (c == '"' || c == '\\') printf("\\%c", c);
        else if (c < 32) printf("\\u%04x", c);
        else putchar(c);
    }
    putchar('"');
}
int main(int argc, char **argv) {
    if (argc != 2) return 2;
    ufbx_load_opts opts = {0};
    ufbx_error error;
    ufbx_scene *scene = ufbx_load_file(argv[1], &opts, &error);
    if (!scene) { fprintf(stderr, "%s\n", error.description.data); return 1; }
    if (scene->anim_stacks.count != 1) { fprintf(stderr, "Expected one animation stack\n"); return 1; }
    size_t total = 0;
    for (size_t i=0; i<scene->anim_curves.count; ++i) total += scene->anim_curves.data[i]->keyframes.count;
    if (!total || total > 1000000) return 1;
    double *times = malloc(total * sizeof(double));
    if (!times) return 1;
    size_t n = 0;
    for (size_t i=0; i<scene->anim_curves.count; ++i) {
        ufbx_anim_curve *c = scene->anim_curves.data[i];
        for (size_t j=0; j<c->keyframes.count; ++j) times[n++] = c->keyframes.data[j].time;
    }
    qsort(times, n, sizeof(double), cmp_time);
    printf("{\"unit_meters\":%.17g,\"fps\":%.17g,\"axes\":[%d,%d,%d],\"nodes\":[",
           scene->settings.unit_meters, scene->settings.frames_per_second,
           scene->settings.axes.right, scene->settings.axes.up, scene->settings.axes.front);
    for (size_t i=0; i<scene->nodes.count; ++i) {
        ufbx_node *node = scene->nodes.data[i];
        if (i) putchar(',');
        printf("{\"name\":"); string_json(node->name.data);
        printf(",\"id\":%u,\"parent_id\":%d,\"rotation_order\":%d}", node->typed_id,
               node->parent ? (int)node->parent->typed_id : -1, (int)node->rotation_order);
    }
    puts("]}");
    for (size_t frame=0; frame<n; ++frame) {
        if (frame && fabs(times[frame]-times[frame-1]) < 1e-9) continue;
        ufbx_scene *evaluated = ufbx_evaluate_scene(scene, scene->anim_stacks.data[0]->anim, times[frame], NULL, &error);
        if (!evaluated) { fprintf(stderr, "%s\n", error.description.data); return 1; }
        printf("{\"time\":%.17g,\"world_matrices\":[", times[frame]);
        for (size_t i=0; i<evaluated->nodes.count; ++i) {
            if (i) putchar(',');
            const ufbx_matrix *m = &evaluated->nodes.data[i]->node_to_world;
            printf("[%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g]",
                   m->m00,m->m01,m->m02,m->m03,m->m10,m->m11,m->m12,m->m13,m->m20,m->m21,m->m22,m->m23);
        }
        puts("]}");
        ufbx_free_scene(evaluated);
    }
    free(times); ufbx_free_scene(scene);
    return ferror(stdout) ? 1 : 0;
}
