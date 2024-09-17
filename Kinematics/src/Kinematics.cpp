#include "Kinematics.h"

std::ostream& CTR::operator<<( std::ostream& str, const Discontinuity& disc)
{
    str << disc.loc << ": " << disc.tube_responsible;
    return str;
}

void CTR::internal::make_normal_points( TInterval::TaggedInterval const& ival, double normal_density, IntegrationPoints& pts )
{
    using namespace TInterval;
    double L = length( ival );

    //2+ is for the endpoints of the interval
    int NPts = 2 + (int)floor( L * normal_density );
    NPts = (NPts > 2) ? NPts : 2;

    Utility::linspace<double> space( left( ival ), right( ival ), NPts );
    auto it_end = space.end(); it_end-=1; //don't add the last point
    std::copy( space.begin(), it_end, std::back_inserter( pts ) );
}

void CTR::internal::make_dense_points( TInterval::TaggedInterval const& ival, double dense_density, IntegrationPoints& pts )
{
    using namespace TInterval;
    double L = length( ival );

    //2+ is for the endpoints of the interval
    int NPts = 2 + (int)ceil( L * dense_density );
    NPts = (NPts > 2) ? NPts : 2;

    Utility::linspace<double> space( left( ival ), right( ival ), NPts );
    auto it_end = space.end(); it_end-=1; //don't add the last point
    std::copy( space.begin(), it_end, std::back_inserter( pts ) );
}

void CTR::internal::make_sparse_points( TInterval::TaggedInterval const& ival, IntegrationPoints& pts )
{
    pts.push_back( TInterval::left( ival ) );
}

void CTR::internal::make_integration_points( TInterval::TaggedInterval const& ival, double normal_density, double dense_density,
                                internal::IntegrationPoints &pts )
{
    using namespace boost::container;

    switch (TInterval::density( ival )) {
    case TInterval::NORMAL:
        internal::make_normal_points( ival, normal_density, pts );
        break;
    case TInterval::DENSE:
        internal::make_dense_points( ival, dense_density, pts );
        break;
    case TInterval::SPARSE:
        internal::make_sparse_points( ival, pts );
        break;
    }

    pts.push_back( right( ival ) );
}

bool CTR::internal::discontinuity_compare( Discontinuity const& disc1, Discontinuity const& disc2 )
{
    return (disc1.loc < disc2.loc);
}

double CTR::internal::get_normal_length( TInterval::IntervalList const& ilist )
{
    using namespace TInterval;
    double L = 0;
    for (auto it = ilist.begin(); it != ilist.end(); ++it) {
        if (has_tag( NORMAL )(*it)) {
           L += TInterval::length( *it );
        }
    }
    return L;
}