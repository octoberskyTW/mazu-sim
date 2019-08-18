#ifndef __DATADECK_HH__
#define __DATADECK_HH__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (DATADECK class)
LIBRARY DEPENDENCY:
      ((../src/datadeck.cpp))
*******************************************************************************/
#include <iostream>
#include <string>
#include <vector>

// sizing of arrays
#define CHARN 768   ///< character numbers in variable names
#define CHARL 1000  ///< character numbers in a line

class Table
{
private:
    std::string name;
    int dim;
    int var1_dim;
    int var2_dim;
    int var3_dim;

public:
    std::vector<double> var1_values;
    std::vector<double> var2_values;
    std::vector<double> var3_values;
    std::vector<double> data;

    Table() {}
    virtual ~Table() {}

    /**
     * @return dimension of table
     * @author 030710 Created by Peter H Zipfel
     */
    int get_dim() { return dim; }


    /**
     * @return name of table
     * @author 030710 Created by Peter H Zipfel
     */
    std::string get_name() { return name; }

    /**
     * @return 1. independent variable dimension
     * @author 030710 Created by Peter H Zipfel
     */
    int get_var1_dim() { return var1_dim; }

    /**
     * Getting 2. independent variable dimension
     * 030710 Created by Peter H Zipfel
     */
    int get_var2_dim() { return var2_dim; }

    /**
     * Getting 3. independent variable dimension
     * 030710 Created by Peter H Zipfel
     */
    int get_var3_dim() { return var3_dim; }

    /**
     * @param[in] table_dim Setting dimension of table
     * @author 030710 Created by Peter H Zipfel
     */
    void set_dim(int table_dim) { dim = table_dim; }

    /**
     * @param[in] tbl_name Setting name of table
     * @author 030710 Created by Peter H Zipfel
     */
    void set_name(std::string tbl_name) { name = tbl_name; }

    /**
     * @param[in] size Setting 1. independent variable dimension
     * @author 030710 Created by Peter H Zipfel
     */
    void set_var1_dim(int size) { var1_dim = size; }

    /**
     * @param[in] size Setting 2. independent variable dimension
     * @author 030710 Created by Peter H Zipfel
     */
    void set_var2_dim(int size) { var2_dim = size; }

    /**
     * @param[in] size Setting 3. independent variable dimension
     * @author 030710 Created by Peter H Zipfel
     */
    void set_var3_dim(int size) { var3_dim = size; }

    /**
     * @param[in] value Setting 1. independent variable values
     * 030710 Created by Peter H Zipfel
     */
    void set_var1_value(int offset, double value)
    {
        var1_values[offset] = value;
    }

    /**
     * @param[in] value Setting 2. independent variable values
     * @author 030710 Created by Peter H Zipfel
     */
    void set_var2_value(int offset, double value)
    {
        var2_values[offset] = value;
    }

    /**
     * @param[in] value Setting 3. independent variable values
     * @author 030710 Created by Peter H Zipfel
     */
    void set_var3_value(int offset, double value)
    {
        var3_values[offset] = value;
    }

    /**
     * @param[in] value Setting tablular data values
     * @author 030710 Created by Peter H Zipfel
     */
    void set_data(int offset, double value)
    {
        data[offset] = value;
    }
};

class Datadeck
{
private:
    std::string title;
    int capacity;
    int tbl_counter;
    std::vector<Table *> table_ptr;

public:
    Datadeck() {}
    explicit Datadeck(const char *file_name);
    virtual ~Datadeck() {}

    /**
     * @brief Allocating memory  table deck title
     * @author 030711 Created by Peter H Zipfel
     */
    void alloc_mem()
    {
        table_ptr = std::vector<Table *>(capacity);
        // for(int i = 0; i < capacity; i++)
        // table_ptr[i] = new Table();
    }

    /**
     * @brief Setting table deck title
     * @author 030711 Created by Peter H Zipfel
     */
    void set_title(std::string deck_title) { title = deck_title; }

    /**
     * @brief Getting table deck title
     * @author 030711 Created by Peter H Zipfel
     */
    std::string get_title() { return title; }

    /**
     * @brief Setting total number of tables
     * @author 030711 Created by Peter H Zipfel
     */
    void set_capacity(int table_numbers) { capacity = table_numbers; }

    /**
     * @brief Getting total number of tables
     * @author 030711 Created by Peter H Zipfel
     */
    int get_capacity() { return capacity; }

    /**
     * @brief Setting table counter
     * @author 030711 Created by Peter H Zipfel
     */
    void set_counter(int count) { tbl_counter = count; }

    /**
     * @brief Getting table counter
     * @author 030711 Created by Peter H Zipfel
     */
    int get_counter() { return tbl_counter; }

    /**
     * @brief Adding a table pointer to the table list
     * @author 030711 Created by Peter H Zipfel
     */
    void add_table(Table &pt)
    {
        if (tbl_counter < capacity) {
            table_ptr[tbl_counter] = &pt;
        }
    }

    /**
     * @brief Overloaded operator [] returns a 'Table' pointer
     * @author 030711 Created by Peter H Zipfel
     */
    Table *operator[](int slot)
    {
        if (slot >= 0 && slot < capacity) {
            return table_ptr[slot];
        } else {
            std::cout << "*** Bad pointer value of table deck: " << slot << '\n';
            return 0;
        }
    }

    /**
     * @brief Getting table pointer
     * @author 030717 Created by Peter H Zipfel
     */
    Table *get_tbl(int slot)
    {
        return table_ptr[slot];
    }

    /**
     * @brief Single independent variable look-up
     * constant extrapolation at the upper end, slope extrapolation at the lower end
     *
     * @author 030717 Created by Peter H Zipfel
     */
    double look_up(std::string name, double value1, unsigned int flag);

    /**
     * @brief Two independent variables look-up
     * Constant extrapolation at the upper end, slope extrapolation at the lower end
     *
     * @author 030717 Created by Peter H Zipfel
     */
    double look_up(std::string name, double value1, double value2, unsigned int flag);

    /**
     * @brief Three independent variables look-up
     * constant extrapolation at the upper end, slope extrapolation at the lower end
     *
     * @author 030723 Created by Peter H Zipfel
     */
    double look_up(std::string name, double value1, double value2, double value3, unsigned int flag);

    /**
     * @brief Table index finder
     * This is a binary search method it is O(lgN)
     * 
     * @returns array locater of the discrete_variable just below variable
     * Keeps max or min array locater if variable is outside those max or min
     *
     * @author 010628 Created by Peter H Zipfel
     */
    int find_index(int max, double value, std::vector<double> list);

    /**
     * @brief Linear one-dimensional interpolation
     * Constant extrapolation beyond max values of X1
     * Slope extrapolation beyond min values of X1
     *
     * @author 030717 Created by Peter H Zipfel
     */
    double interpolate(int ind, int ind2, int slot, double val);

    /**
     * @brief Linear, two-dimensional interpolation
     * Constant extrapolation beyond max values of X1 and X2
     * Slope extrapolation beyond min values of X1 and X2
     *
     * @author 030718 Created by Peter H Zipfel
     */
    double interpolate(int ind10, int ind11, int ind20, int ind21, int slot, double value1,
                       double value2);

    /**
     * @brief Linear, three-dimensional interpolation
     * Constant extrapolation beyond max values of X1, X2 and X3
     * Slope extrapolation beyond min values of X1, X2 and X3
     *
     * @author 030723 Created by Peter Zipfel
     */
    double interpolate(int ind10, int ind11, int ind20, int ind21, int ind30, int ind31,
                       int slot, double value1, double value2, double value3);
};

#endif  // __DATADECK_HH__
