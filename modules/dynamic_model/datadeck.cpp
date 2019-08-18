#include "datadeck.hh"

#include <armadillo>

#include <iostream>

/**
 * @brief Read table & store table's data
 */
Datadeck::Datadeck(const char *file_name)
{
    char line_clear[CHARL];
    char temp[CHARN];  // buffer for table data
    std::string table_deck_title;
    int table_count(0);
    double value(0);
    int var_dim[3] = { 1, 1, 1 };
    int tt(0);
    Table *table;
    // opening aero-deck file stream
    std::ifstream tbl_stream(file_name);

    if (tbl_stream.fail()) {
        std::cerr << "*** Error: File stream '" << file_name << "' failed to open (check spelling) ***\n";
        exit(1);
    }

    // determing the total # of tbl_stream
    while (!tbl_stream.eof()) {
        tbl_stream >> temp;
        if (!strcmp(temp, "TITLE")) {
            tbl_stream.getline(line_clear, CHARL, '\n');
            table_deck_title = line_clear;
        }
        if (strstr(temp, "DIM"))
            table_count++;
    }  // EOF reached of aero_deck

    // removing EOF bit (-1)
    tbl_stream.clear();
    // rewinding to beginning
    tbl_stream.seekg(std::ios::beg);

    // creating pointer array table
    this->set_title(table_deck_title);
    this->set_capacity(table_count);
    this->alloc_mem();

    // discarding all entries until first DIM
    do {
        tbl_stream >> temp;
    } while (!strstr(temp, "DIM"));

    // loading tables one at a time
    for (int t = 0; t < table_count; t++) {
        // creating and allocating memory to object 'table'
        table = new Table;

        // extracting table dimension
        // at this point 'temp' is holding xDIM
        char dim_buff[2];
        int table_dim(0);
        strncpy(dim_buff, temp, 1);
        dim_buff[1] = '\0';
        // converting character to integer
        sscanf(dim_buff, "%d", &table_dim);
        table->set_dim(table_dim);

        // extracting table name
        tbl_stream >> temp;
        table->set_name(temp);
        tbl_stream.getline(line_clear, CHARL, '\n');

        // extracting dimensions of independent variables
        var_dim[0] = 1;
        var_dim[1] = 1;
        var_dim[2] = 1;
        for (tt = 0; tt < table_dim; tt++) {
            tbl_stream >> temp;
            tbl_stream >> var_dim[tt];
            if (tt == 0)
                table->set_var1_dim(var_dim[tt]);
            if (tt == 1)
                table->set_var2_dim(var_dim[tt]);
            if (tt == 2)
                table->set_var3_dim(var_dim[tt]);
        }
        tbl_stream.getline(line_clear, CHARL, '\n');

        // allocating memory for variables and data arrays
        table->var1_values = std::vector<double>(var_dim[0]);
        table->var2_values = std::vector<double>(var_dim[1]);
        table->var3_values = std::vector<double>(var_dim[2]);
        table->data = std::vector<double>(var_dim[0] * var_dim[1] * var_dim[2]);

        // determining max number of rows of data
        int num_rows = var_dim[0];
        if (var_dim[0] < var_dim[1])
            num_rows = var_dim[1];
        if (var_dim[2] > num_rows)
            num_rows = var_dim[2];

        // reading num_row of data
        for (tt = 0; tt < num_rows; tt++) {
            // loading 1.variable values
            if (tt < var_dim[0]) {
                tbl_stream >> value;
                table->set_var1_value(tt, value);
            }

            // loading 2.variable values, but bypass if default dimension one
            if (tt < var_dim[1] && var_dim[1] != 1) {
                tbl_stream >> value;
                table->set_var2_value(tt, value);
            }

            // loading 3.variable values, but bypass if default dimension one
            if (tt < var_dim[2] && var_dim[2] != 1) {
                tbl_stream >> value;
                table->set_var3_value(tt, value);
            }

            // loading tabular data, which in all cases has only 'var_dim[0]' rows
            if (tt < var_dim[0]) {
                // read one row of data
                for (int ttt = 0; ttt < var_dim[1] * var_dim[2]; ttt++) {
                    tbl_stream >> value;
                    table->set_data(tt * var_dim[1] * var_dim[2] + ttt, value);
                }
            }
        }  // end of reading data

        // loading table into 'Datadeck' pointer array 'Table **tabel_ptr'
        this->set_counter(t);
        this->add_table(*table);
        tbl_stream >> temp;  // reading next DIM entry
    }                        // end of 'for' loop, finished loading all tables
}

/**
 * @brief Single independent variable look-up
 * constant extrapolation at the upper end, slope extrapolation at the lower end
 * flag = 1 means if input value hit upper bound use max value 
 * @author 030717 Created by Peter H Zipfel
 */
double Datadeck::look_up(std::string name, double value1, unsigned int flag)
{
    // finding slot of table in table pointer array (Table **table_ptr)
    int slot(-1);
    std::string tbl_name;
    do {
        slot++;
        tbl_name = get_tbl(slot)->get_name();
    } while (name != tbl_name);

    // getting table index locater of discrete value just below of variable value
    int var1_dim = get_tbl(slot)->get_var1_dim();
    int loc1 = find_index(var1_dim - 1, value1, get_tbl(slot)->var1_values);
    if (flag == 1) {
        if (loc1 == (var1_dim - 1))
            value1 = get_tbl(slot)->var1_values[loc1];
        if (loc1 == 0) {
            if (value1 < get_tbl(slot)->var1_values[loc1])
                value1 = get_tbl(slot)->var1_values[loc1];
        }
    }

    // using max discrete value if value is outside table
    // if (loc1 == (var1_dim-1)) return get_tbl(slot)->data[loc1];
    if (loc1 == (var1_dim - 1))
        return interpolate(loc1 - 1, loc1, slot, value1);

    return interpolate(loc1, loc1 + 1, slot, value1);
}

/**
 * @brief Two independent variables look-up
 * Constant extrapolation at the upper end, slope extrapolation at the lower end
 * flag = 1 means if input value hit upper bound use max value 
 * @author 030717 Created by Peter H Zipfel
 */
double Datadeck::look_up(std::string name, double value1, double value2, unsigned int flag)
{
    // finding slot of table in table pointer array (Table **table_ptr)
    int slot(-1);
    std::string tbl_name;
    do {
        slot++;
        tbl_name = get_tbl(slot)->get_name();
    } while (name != tbl_name);


    // getting table index (off-set) locater of discrete value just below or equal of the variable value
    int var1_dim = get_tbl(slot)->get_var1_dim();
    int loc1 = find_index(var1_dim - 1, value1, get_tbl(slot)->var1_values);

    int var2_dim = get_tbl(slot)->get_var2_dim();
    int loc2 = find_index(var2_dim - 1, value2, get_tbl(slot)->var2_values);

    if (flag == 1) {
        if (loc1 == (var1_dim - 1))
            value1 = get_tbl(slot)->var1_values[loc1];
        if (loc2 == (var2_dim - 1))
            value2 = get_tbl(slot)->var2_values[loc2];
        if (loc1 == 0) {
            if (value1 < get_tbl(slot)->var1_values[loc1])
                value1 = get_tbl(slot)->var1_values[loc1];
        }
        if (loc2 == 0) {
            if (value2 < get_tbl(slot)->var2_values[loc2])
                value2 = get_tbl(slot)->var2_values[loc2];
        }
    }

    if (loc1 == (var1_dim - 1) && loc2 == (var2_dim - 1)) {
        return interpolate(loc1 - 1, loc1, loc2 - 1, loc2, slot, value1, value2);
    } else if (loc2 == (var2_dim - 1)) {
        return interpolate(loc1, loc1 + 1, loc2 - 1, loc2, slot, value1, value2);
    } else if (loc1 == (var1_dim - 1)) {
        return interpolate(loc1 - 1, loc1, loc2, loc2 + 1, slot, value1, value2);
    } else {
        return interpolate(loc1, loc1 + 1, loc2, loc2 + 1, slot, value1, value2);
    }
}

/**
 * @brief Three independent variables look-up
 * constant extrapolation at the upper end, slope extrapolation at the lower end
 * flag = 1 means if input value hit upper bound use max value 
 * @author 030723 Created by Peter H Zipfel
 */
double Datadeck::look_up(std::string name, double value1, double value2, double value3, unsigned int flag)
{
    // finding slot of table in table pointer array (Table **table_ptr)
    int slot(-1);
    std::string tbl_name;
    do {
        slot++;
        tbl_name = get_tbl(slot)->get_name();
    } while (name != tbl_name);


    // getting table index locater of discrete value just below of variable value
    int var1_dim = get_tbl(slot)->get_var1_dim();
    int loc1 = find_index(var1_dim - 1, value1, get_tbl(slot)->var1_values);

    int var2_dim = get_tbl(slot)->get_var2_dim();
    int loc2 = find_index(var2_dim - 1, value2, get_tbl(slot)->var2_values);

    int var3_dim = get_tbl(slot)->get_var3_dim();
    int loc3 = find_index(var3_dim - 1, value3, get_tbl(slot)->var3_values);

    if (flag == 1) {
        if (loc1 == (var1_dim - 1))
            value1 = get_tbl(slot)->var1_values[loc1];
        if (loc2 == (var2_dim - 1))
            value2 = get_tbl(slot)->var2_values[loc2];
        if (loc3 == (var3_dim - 1))
            value3 = get_tbl(slot)->var3_values[loc3];
        if (loc1 == 0) {
            if (value1 < get_tbl(slot)->var1_values[loc1])
                value1 = get_tbl(slot)->var1_values[loc1];
        }
        if (loc2 == 0) {
            if (value2 < get_tbl(slot)->var2_values[loc2])
                value2 = get_tbl(slot)->var2_values[loc2];
        }
        if (loc3 == 0) {
            if (value3 < get_tbl(slot)->var3_values[loc3])
                value3 = get_tbl(slot)->var3_values[loc3];
        }
    }

    if (loc1 == (var1_dim - 1) && loc2 == (var2_dim - 1) && loc3 == (var3_dim - 1)) {
        return interpolate(loc1 - 1, loc1, loc2 - 1, loc2, loc3 - 1, loc3, slot, value1, value2, value3);
    } else if (loc3 == (var3_dim - 1)) {
        return interpolate(loc1, loc1 + 1, loc2, loc2 + 1, loc3 - 1, loc3, slot, value1, value2, value3);
    } else if (loc2 == (var2_dim - 1)) {
        return interpolate(loc1, loc1 + 1, loc2 - 1, loc2, loc3, loc3 + 1, slot, value1, value2, value3);
    } else if (loc1 == (var1_dim - 1)) {
        return interpolate(loc1 - 1, loc1, loc2, loc2 + 1, loc3, loc3 + 1, slot, value1, value2, value3);
    } else if (loc3 == (var3_dim - 1) && loc2 == (var2_dim - 1)) {
        return interpolate(loc1, loc1 + 1, loc2 - 1, loc2, loc3 - 1, loc3, slot, value1, value2, value3);
    } else if (loc3 == (var3_dim - 1) && loc1 == (var1_dim - 1)) {
        return interpolate(loc1 - 1, loc1, loc2, loc2 + 1, loc3 - 1, loc3, slot, value1, value2, value3);
    } else if (loc1 == (var1_dim - 1) && loc2 == (var2_dim - 1)) {
        return interpolate(loc1 - 1, loc1, loc2 - 1, loc2, loc3, loc3 + 1, slot, value1, value2, value3);
    } else {
        return interpolate(loc1, loc1 + 1, loc2, loc2 + 1, loc3, loc3 + 1, slot, value1, value2, value3);
    }
}

/**
 * @brief Table index finder
 * This is a binary search method it is O(lgN)
 * 
 * @returns array locater of the discrete_variable just below variable
 * Keeps max or min array locater if variable is outside those max or min
 *
 * @author 010628 Created by Peter H Zipfel
 */
int Datadeck::find_index(int max, double value, std::vector<double> list)
{
    if (value >= list[max]) {
        return max;
    } else if (value <= list[0]) {
        return 0;
    } else {
        int index = 0;
        int mid;
        while (index <= max) {
            mid = (index + max) / 2;  // integer division
            if (value < list[mid])
                max = mid - 1;
            else if (value > list[mid])
                index = mid + 1;
            else
                return mid;
        }
        return max;
    }
}

/**
 * @brief Linear one-dimensional interpolation
 * Data deck must contain table in the following format:
 *
 * X1       Table Values(X1)                            
 *                                                      
 * X11      Y11
 * X12      Y12                                         
 * X13      Y13                                         
 *
 * Constant extrapolation beyond max values of X1
 * Slope extrapolation beyond min values of X1
 *
 * @author 030717 Created by Peter H Zipfel
 */
double Datadeck::interpolate(int ind1, int ind2, int slot, double val)
{
    double dx(0), dy(0);
    double dumx(0);

    double diff = val - get_tbl(slot)->var1_values[ind1];
    dx = get_tbl(slot)->var1_values[ind2] - get_tbl(slot)->var1_values[ind1];
    dy = get_tbl(slot)->data[ind2] - get_tbl(slot)->data[ind1];

    if (fabs(dx) > arma::datum::eps)
        dumx = diff / dx;
    dy = dumx * dy;

    return get_tbl(slot)->data[ind1] + dy;
}

/**
 * @brief Linear, two-dimensional interpolation
 * File must contain table in the following form:
 *
 *  X1  X2  //Table Values(X1-row, X2-column)
 *            ---------------
 *  X11 X21   |Y11  Y12  Y13|
 *  X12 X22   |Y21  Y22  Y23|    <- data
 *  X13 X23   |Y31  Y32  Y33|
 *  X14       |Y41  Y42  Y43|
 *            ---------------
 * Constant extrapolation beyond max values of X1 and X2
 * Slope extrapolation beyond min values of X1 and X2
 *
 * @author 030718 Created by Peter H Zipfel
 */
double Datadeck::interpolate(int ind10, int ind11, int ind20, int ind21, int slot, double value1,
                             double value2)
{
    double dx1(0), dx2(0);
    double dumx1(0), dumx2(0);

    int var1_dim = get_tbl(slot)->get_var1_dim();
    int var2_dim = get_tbl(slot)->get_var2_dim();

    double diff1 = value1 - get_tbl(slot)->var1_values[ind10];
    double diff2 = value2 - get_tbl(slot)->var2_values[ind20];

    // if (ind10 == (var1_dim-1))  // Assures constant upper extrapolation of first variable
    //     ind11 = ind10;
    // else
    dx1 = get_tbl(slot)->var1_values[ind11] - get_tbl(slot)->var1_values[ind10];

    // if (ind20 == (var2_dim-1))  // Assures constant upper extrapolation of second variable
    //     ind21 = ind20;
    // else
    dx2 = get_tbl(slot)->var2_values[ind21] - get_tbl(slot)->var2_values[ind20];

    if (fabs(dx1) > arma::datum::eps)
        dumx1 = diff1 / dx1;
    if (fabs(dx2) > arma::datum::eps)
        dumx2 = diff2 / dx2;

    double y11 = get_tbl(slot)->data[ind10 * var2_dim + ind20];
    double y12 = get_tbl(slot)->data[ind10 * var2_dim + ind21];
    double y21 = get_tbl(slot)->data[ind11 * var2_dim + ind20];
    double y22 = get_tbl(slot)->data[ind11 * var2_dim + ind21];
    double y1 = dumx1 * (y21 - y11) + y11;
    double y2 = dumx1 * (y22 - y12) + y12;

    return dumx2 * (y2 - y1) + y1;
}
/**
 * @brief Linear, three-dimensional interpolation
 * File must contain table in the following form:
 *
 *  X1  X2  X3    Table Values(X1-row, X2-block, X3-column) <- don't type (illustration only)
 *
 *                (X1 x X3) (X1 x X3) (X1 x X3) (X1 x X3)   <- don't type
 *                 for X21   for X22   for X23   for X24    <- don't type
 *               -----------------------------------------
 *  X11 X21 X31  |Y111 Y112|Y121 Y122|Y131 Y132|Y141 Y142|
 *  X12 X22 X32  |Y211 Y212|Y221 Y222|Y231 Y232|Y241 Y242|  <- data; don't type: '|'
 *  X13 X23      |Y311 Y312|Y321 Y322|Y331 Y332|Y341 Y342|
 *      X24      -----------------------------------------
 *
 * Constant extrapolation beyond max values of X1, X2 and X3
 * Slope extrapolation beyond min values of X1, X2 and X3
 *
 * @author 030723 Created and corrected by Peter Zipfel
 */
double Datadeck::interpolate(int ind10, int ind11, int ind20, int ind21, int ind30, int ind31,
                             int slot, double value1, double value2, double value3)
{
    double dx1(0), dx2(0), dx3(0);
    double dumx1(0), dumx2(0), dumx3(0);

    int var1_dim = get_tbl(slot)->get_var1_dim();
    int var2_dim = get_tbl(slot)->get_var2_dim();
    int var3_dim = get_tbl(slot)->get_var3_dim();

    double diff1 = value1 - get_tbl(slot)->var1_values[ind10];
    double diff2 = value2 - get_tbl(slot)->var2_values[ind20];
    double diff3 = value3 - get_tbl(slot)->var3_values[ind30];

    if (ind10 != (var1_dim - 1))  // Assures constant upper extrapolation of first variable
        dx1 = get_tbl(slot)->var1_values[ind11] - get_tbl(slot)->var1_values[ind10];

    if (ind20 == (var2_dim - 1))  // Assures constant upper extrapolation of second variable
        ind21 = ind20;
    else
        dx2 = get_tbl(slot)->var2_values[ind21] - get_tbl(slot)->var2_values[ind20];

    if (ind30 == (var3_dim - 1))  // Assures constant upper extrapolation of third variable
        ind31 = ind30;
    else
        dx3 = get_tbl(slot)->var3_values[ind31] - get_tbl(slot)->var3_values[ind30];

    if (dx1 > arma::datum::eps)
        dumx1 = diff1 / dx1;
    if (dx2 > arma::datum::eps)
        dumx2 = diff2 / dx2;
    if (dx3 > arma::datum::eps)
        dumx3 = diff3 / dx3;
    // int ind10,int ind11,int ind20,int ind21,int ind30,int ind31
    //      i        i+1        j         j+1       k        k+1
    // Use innner x1 and outer variable x3 for 2DIM interpolation, middle variable x2 is parameter
    // For parameter ind20
    double y11 = get_tbl(slot)->data[ind10 * var2_dim * var3_dim + ind20 * var3_dim + ind30];
    double y12 = get_tbl(slot)->data[ind10 * var2_dim * var3_dim + ind20 * var3_dim + ind30 + var2_dim * var3_dim];
    double y31 = get_tbl(slot)->data[ind10 * var2_dim * var3_dim + ind20 * var3_dim + ind31];
    double y32 = get_tbl(slot)->data[ind10 * var2_dim * var3_dim + ind20 * var3_dim + ind31 + var2_dim * var3_dim];
    // 2DIM interpolation
    double y1 = dumx1 * (y12 - y11) + y11;
    double y3 = dumx1 * (y32 - y31) + y31;
    double y21 = dumx3 * (y3 - y1) + y1;

    // For parameter ind21
    y11 = get_tbl(slot)->data[ind10 * var2_dim * var3_dim + ind21 * var3_dim + ind30];
    y12 = get_tbl(slot)->data[ind10 * var2_dim * var3_dim + ind21 * var3_dim + ind30 + var2_dim * var3_dim];
    y31 = get_tbl(slot)->data[ind10 * var2_dim * var3_dim + ind21 * var3_dim + ind31];
    y32 = get_tbl(slot)->data[ind10 * var2_dim * var3_dim + ind21 * var3_dim + ind31 + var2_dim * var3_dim];
    // 2DIM interpolation
    y1 = dumx1 * (y12 - y11) + y11;
    y3 = dumx1 * (y32 - y31) + y31;
    double y22 = dumx3 * (y3 - y1) + y1;

    // 1DIM interpolation between the middle variable
    return dumx2 * (y22 - y21) + y21;
}
