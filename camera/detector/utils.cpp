#include <stdio.h>
#include <stdlib.h>
#include "utils.h"

static unsigned char *LoadData(FILE *fp, size_t ofst, size_t sz)
{
    unsigned char *data;
    int ret;

    data = NULL;

    if (NULL == fp)
    {
        return NULL;
    }

    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0)
    {
        printf("blob seek failure.\n");
        return NULL;
    }

    data = (unsigned char *)malloc(sz);
    if (data == NULL)
    {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
    return data;
}

unsigned char *LoadModel(const char *filename, int *model_size)
{
    FILE *fp;
    unsigned char *data;

    fp = fopen(filename, "rb");
    if (NULL == fp)
    {
        printf("Open file %s failed.\n", filename);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);

    data = LoadData(fp, 0, size);

    fclose(fp);

    *model_size = size;
    return data;
}

static char* ReadLine(FILE* fp, char* buffer, int* len)
{
	int    ch;
	int    i        = 0;
	size_t buff_len = 0;

	buffer = (char*)malloc(buff_len + 1);
	if (!buffer)
		return NULL; // Out of memory

	while ((ch = fgetc(fp)) != '\n' && ch != EOF) {
		buff_len++;
		void* tmp = realloc(buffer, buff_len + 1);
		if (tmp == NULL) {
            free(buffer);
            return NULL; // Out of memory
		}
		buffer = (char*)tmp;

		buffer[i] = (char)ch;
		i++;
	}
	buffer[i] = '\0';

	*len = buff_len;

	// Detect end
	if (ch == EOF && (i == 0 || ferror(fp))) {
		free(buffer);
		return NULL;
	}
	return buffer;
}

int LoadLabelName(const char* locationFilename, std::vector<std::string>& labels)
{
	// printf("loadLabelName %s\n", locationFilename);

	FILE* file = fopen(locationFilename, "r");
	char* s;
	int   i = 0;
	int   n = 0;

	if (file == NULL) {
		printf("Open %s fail!\n", locationFilename);
		return -1;
	}

	while ((s = ReadLine(file, s, &n)) != NULL) {
		// lines[i++] = s;
        labels.push_back(s);
	}
	fclose(file);

	return 0;
}


