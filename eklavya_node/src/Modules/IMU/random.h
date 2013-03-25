#include <stdlib.h>
#include <string.h>
#include <stdio.h>

char substring(char s[], char sub[], int ini, int len) {

    for (int i = 0; i < len; i++)
        sub[i] = s[i + ini];

    return ('0');
}

int LastIndexOf(char s[], char c) {
    int ind = 0;
    int len = strlen(s);
    for (int i = 0; i < len; i++) {
        if (s[i] == c)
            ind = i;
    }
    return ind;
}
