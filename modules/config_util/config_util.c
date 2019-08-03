#include "config_util.h"

int get_enum_value(const char *enumstr)
{
    fprintf(stdout, "%s\n", enumstr);
    return 0;
}

int read_table_file(void *table, FILE *fp, int *numEntries, char *specifier)
{
    int rc;
    char line[256];
    *numEntries = 0;
    // No one's going to have more than 32 columns... right?
    CONFIG_SPECIFIER_E fieldTypes[32];
    int fieldSizes[32];
    int numFields = 0;
    int structSize = 0;
    // Start by parsing the specifier string
    // Format: "ENUM:4 STRING:32 NUMBER:4"

    // Split by spaces
    char *variable = strtok_r(specifier, " ", &specifier);
    while (variable != NULL) {
        // Split by colons and then parse the byte size
        char *type = strtok_r(variable, ":", &variable);
        char *sizeString = strtok_r(NULL, ":", &variable);
        int size = (int) strtol(sizeString, (char **) NULL, 0);
        // Set the type
        if (0 == strcmp(type, "STRING")) {
            fieldTypes[numFields] = CONFIG_STRING;
        } else if (0 == strcmp(type, "ENUM")) {
            fieldTypes[numFields] = CONFIG_ENUM;
        } else if (0 == strcmp(type, "NUMBER")) {
            fieldTypes[numFields] = CONFIG_NUMBER;
        } else {
            // Unrecognized type, cannot continue
            fprintf(
                stderr,
                "ERROR: unrecognized specifier type %s! Please check your\n",
                type);
            fprintf(stderr,
                    " input or check read_config.cpp. Available types: "
                    "STRING/ENUM/NUMBER\n");
            return CONFIG_ERROR;
        }
        // Size of the field in bytes
        fieldSizes[numFields] = size;
        structSize += size;
        numFields++;
        // Split for the next loop
        variable = strtok_r(NULL, " ", &specifier);
    }

    // Now actually read the config file
    int i = 0;
    int linenum = 0;
    while (1) {
        // Scan a line
        rc = fscanf(fp, "%[^\n]\n", line);
        if (EOF == rc)
            break;
        linenum++;
        // Skip comment lines
        if ('#' == line[0])
            continue;
        // Replace first '#' with '\0' to ignore in-line comments
        int j = 0;
        while (line[j] != '\0' && j < 256) {
            if ('#' == line[j]) {
                line[j] = '\0';
                break;
            }
            j++;
        }
        /*
         * Parse the line. Can't use sscanf because of the variable pointers...
         */
        int numFieldsRead = 0;
        int enumVal;
        int parsedVal;
        // Need to read in all of them first to know how many there are
        char *fieldArray[256];
        // It doesn't like it when it's the original string, so use a copy
        char lineCopy[256];
        strcpy(lineCopy, line);
        char *strptr = lineCopy;
        char *field = strtok_r(strptr, " ", &strptr);
        int error = 0;
        while (field != NULL) {
            fieldArray[numFieldsRead] = field;
            numFieldsRead++;
            field = strtok_r(NULL, " ", &strptr);
        }  // while (field != NULL)

        // Got expected number of fields from the line
        if (numFieldsRead == numFields) {
            // Now do the filling out
            // The current pointer so we can access the struct dynamically by
            // memory
            char *currPointer = (char *) table + structSize * i;
            for (numFieldsRead = 0; numFieldsRead < numFields;
                 numFieldsRead++) {
                field = fieldArray[numFieldsRead];
                // Do not set values if too many fields, to avoid overrunning
                // buffer
                if (fieldSizes[numFieldsRead] > 0) {
                    // Fill out the appropriate field in the struct
                    switch (fieldTypes[numFieldsRead]) {
                    case CONFIG_STRING:
                        // Copy 1 smaller than the size for the null-terminated
                        // string
                        strncpy(currPointer, field,
                                fieldSizes[numFieldsRead] - 1);
                        break;
                    case CONFIG_ENUM:
                        // Translate name of enum to enum value, which is an
                        // integer
                        enumVal = get_enum_value(field);
                        if (enumVal == -1) {
                            fprintf(stderr,
                                    "ERROR: Aborting. Do not pass Go.\n");
                            return CONFIG_ERROR;
                        }
                        *(int *) currPointer = enumVal;
                        break;
                    case CONFIG_NUMBER:
                        // Numbers should be easy
                        parsedVal = (int) strtol(field, (char **) NULL, 0);
                        *(int *) currPointer = parsedVal;
                        break;
                    default:
                        // This shouldn't happen, would be something wrong with
                        // memory maybe?
                        fprintf(stderr,
                                "ERROR: invalid type for the field (check "
                                "read_config.cpp code)\n");
                        fprintf(stderr,
                                "ERROR: Aborting. Do not pass Go. Do not "
                                "collect $200.\n");
                        return CONFIG_ERROR;
                        break;
                    }  // switch (fieldTypes[numFieldsRead])
                }      // if (numFieldsRead < numFields)
                // Move the pointer for the next field
                currPointer += fieldSizes[numFieldsRead];
            }
        } else {
            // Didn't get correct number of fields, but this may be intended
            // (like for atomintr)
            fprintf(
                stderr,
                "WARNING: Expected %d fields; got %d while reading line %d\n",
                numFields, numFieldsRead, linenum);
            error++;
        }
        // Don't increment i if it failed, so the next will overwrite properly
        if (!error)
            i++;
    }
    *numEntries = i;
    return CONFIG_SUCCESS;
}  // read_table_file()

int read_config(void *table, int *numEntries, const char *config_path,
                char *specifier)
{
    int rc = CONFIG_SUCCESS;
    char table_file[256];
    FILE *fp;

    sprintf(table_file, "%s", config_path);
    fp = fopen(table_file, "r");
    if (NULL == fp) {
        fprintf(stderr, "ERROR: Could not find the %s\n", config_path);
        rc = CONFIG_ERROR;
    }

    if (NULL != fp) {
        fprintf(stdout, "Reading from %s \n", config_path);
        rc = read_table_file(table, fp, numEntries, specifier);
        fprintf(stdout, "%d entries from config file %s\n", *numEntries,
                config_path);
        fclose(fp);
    }
    return rc;
}