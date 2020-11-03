import jinja2
from astropy.utils import iers


template = jinja2.Template(
    """
#ifndef __DM_DELTA_UT_HH__
#define __DM_DELTA_UT_HH__

/*
 * DO NOT EDIT THIS FILE
 *
 * This file is generate by script
 *
 * Reference:
 *   - https://datacenter.iers.org/eop.php
 *   - https://docs.astropy.org/en/stable/utils/iers.html
 *
 * Start date: 20{{ start_date }}
 * End date  : 20{{ end_date }}
 */

#define MAX_DM_UT1_UT_INDEX {{ length }}
#define DM_UT1_UT_BASE_MJD {{ base_mjd }}

const double DM_UT1_UT[MAX_DM_UT1_UT_INDEX] = {
    {% for dut1_row in dut1 | batch(5) -%}
        {{ dut1_row | join(", ") }},
    {% endfor -%}
};

#endif /* __DM_DELTA_UT_HH__ */
"""
)
dat = iers.earth_orientation_table.get()[-4000:]
dut1 = dat["UT1_UTC"]
start_date = "/".join(map(str, dat["year", "month", "day"][0]))
end_date = "/".join(map(str, dat["year", "month", "day"][-1]))

print(
    template.render(
        start_date=start_date,
        end_date=end_date,
        length=len(dat),
        base_mjd=dat["MJD"][0].to_value(),
        dut1=list(map(lambda x: x.to_value(), dut1)),
    )
)
