#ifndef OCS_MEDIAN_H
#define OCS_MEDIAN_H

#include "OCS_RobustWindowFilter.h"

// Deprecated compatibility wrapper.
//
// OcsMedianFilter is kept only to avoid breaking older call sites that
// still include ocs_median.h or instantiate OcsMedianFilter.
//
// OcsMedianFilter is an alias for OCS_RobustWindowFilter. Treat the
// implementation as the robust window filter contract, not as a pure
// median-filter API contract implied by the legacy name.
//
// New code should include OCS_RobustWindowFilter.h directly and use
// OCS_RobustWindowFilter.
using OcsMedianFilter = OCS_RobustWindowFilter;

#endif // OCS_MEDIAN_H
