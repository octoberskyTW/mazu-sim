#ifndef __CADAC_UTIL_HH__
#define __CADAC_UTIL_HH__

#include <armadillo>
#include <tuple>

namespace cad
{
/**
     * @return great circle distance between two point on a spherical Earth
     * @brief Refrence: Bate et al. "Fundamentals of Astrodynamics", Dover 1971, p. 310
     *
     * @param[in] lon1 longitude of first  point - rad
     * @param[in] lat1 latitude  of first  point - rad
     * @param[in] lon2 longitude of second point - rad
     * @param[in] lat2 latitude  of second point - rad
     *
     * @author 030414 Created from FORTRAN by Peter H Zipfel
     */


double distance(const double &lon1,
                const double &lat1,
                const double &lon2,
                const double &lat2);

/**
     * @brief Calculates geodetic longitude, latitude, and altitude from inertial
     *        displacement vector
     *        using the WGS 84 reference ellipsoid
     *        Reference: Britting,K.R."Inertial Navigation Systems Analysis", Wiley. 1971
     *
     * @return std::tuple(lon, lat, alt)
     *                    lon geodetic longitude - rad
     *                    lat geodetic latitude - rad
     *                    alt altitude above ellipsoid - m
     *
     * @param[in] SBII(3x1) = Inertial position - m
     *
     * @author 030414 Created from FORTRAN by Peter H Zipfel
     * @author 170121 Create Armadillo Version by soncyang
     */
std::tuple<double, double, double> geo84_in(arma::vec3 SBII,
                                            arma::mat33 TEI);

/**
     * @return geodetic velocity vector information from inertial postion and
     * velocity
     * using the WGS 84 reference ellipsoid
     *
     * @brief Calls utilities
     *    geo84_in(...), tdi84(...)
     *
     * @param[out] dvbe geodetic velocity - m/s
     * @param[out] psivdx geodetic heading angle - deg
     * @param[out] thtvdx geodetic flight path angle - deg
     *
     * @param[in] SBII(3x1) Inertial position - m
     * @param[in] VBII(3x1) Inertial velocity - m
     *
     * @author 040710 created by Peter H Zipfel
     */
std::tuple<double, double, double> geo84vel_in(arma::vec3 SBII,
                                               arma::vec3 VBII,
                                               arma::mat33 TEI);
/**
     * @return geocentric lon, lat, alt from inertial displacement vector
     * for spherical Earth
     *
     * @param[out] lonc = geocentric longitude - rad
     * @param[out] latc = geocentric latitude - rad
     * @param[out] altc = geocentric latitude - m
     *
     * @param[in] SBII = Inertial position - m
     * @param[in] time = simulation time - sec
     *
     * @author 010628 Created by Peter H Zipfel
     * @author 030416 Modified for SBII (not SBIE) input, PZi
     */
std::tuple<double, double, double> geoc_in(arma::vec3 SBII,
                                           const double &time);

/**
     * @return lon, lat, alt from displacement vector in Earth coord for spherical
     * earth
     * RETURN[0]=lon
     * RETURN[1]=lat
     * RETURN[2]=alt
     *
     * @param[in] SBIE = displacement of vehicle wrt Earth center in Earth coordinates
     *
     * @author 010628 Created by Peter H Zipfel
     */
std::tuple<double, double, double> geoc_ine(arma::vec3 SBIE);

/**
     * @brief Earth gravitational acceleration, using the WGS 84 ellipsoid
     *      Ref: Chatfield, A.B.,"Fundamentals of High Accuracy Inertial
     *      Navigation",p.10, Prog.Astro and Aeronautics, Vol 174, AIAA, 1997.
     *
     * @return GRAVG(3x1) = gravitational acceleration in geocentric coord - m/s^2
     *
     * @param[in] SBII = inertial displacement vector - m
     * @param[in] time = simulation time - sec
     *
     * @author 030417 Created from FORTRAN by Peter H Zipfel
     */
arma::vec3 grav84(arma::vec3 SBII, const double &time);

/**
     * @brief Returns the inertial displacement vector from longitude, latitude and
     *      altitude
     *      using the WGS 84 reference ellipsoid
     *      Reference: Britting,K.R."Inertial Navigation Systems Analysis"
     *      pp.45-49, Wiley, 1971
     *
     * @return SBII(3x1) = Inertial vehicle position - m
     *
     * @param[in] lon = geodetic longitude - rad
     * @param[in] lat = geodetic latitude - rad
     * @param[in] alt = altitude above ellipsoid - m
     * @param[in] time = simulation time - sec
     *
     * @author 030411 Created from FORTRAN by Peter H Zipfel
     * @author 170121 Create Armadillo Version by soncyanga
     */
arma::vec3 in_geo84(const double lon,
                    const double lat,
                    const double alt,
                    arma::mat33 TEI);

/**
     * @brief Returns the inertial displacement vector from geocentric longitude, latitude
     *      and altitude
     *      for spherical Earth
     *
     * @return SBII = position of vehicle wrt center of Earth, in inertial coord
     *
     * @param[in] lon = geographic longitude - rad
     * @param[in] lat = geocentric latitude - rad
     * @param[in] alt = altitude above spherical Earth = m
     *
     * @author 010405 Created by Peter H Zipfel
     */
arma::vec3 in_geoc(const double &lon,
                   const double &lat,
                   const double &alt,
                   const double &time);

/**
     * @brief Calculates inertial displacement and velocity vectors from orbital elements
     *      Reference: Bate et al. "Fundamentals of Astrodynamics", Dover 1971, p.71
     *
     * @return parabola_flag = 0 ok
     *                       = 1 not suitable (divide by zero), because parabolic
     *                           trajectory
     *
     * @param[out] SBII = Inertial position - m
     * @param[out] SBII = Inertial velocity - m/s
     *
     * @param[in] semi = semi-major axis of orbital ellipsoid - m
     * @param[in] ecc = eccentricity of elliptical orbit - ND
     * @param[in] inclx = inclination of orbital wrt equatorial plane - deg
     * @param[in] lon_anodex = celestial longitude of the ascending node - deg
     * @param[in] arg_perix = argument of periapsis (ascending node to periapsis) - deg
     * @param[in] true_anomx = true anomaly (periapsis to satellite) - deg
     *
     * @author 040510 Created by Peter H Zipfel
     */
int in_orb(arma::vec3 &SBII,
           arma::vec3 &VBII,
           const double &semi,
           const double &ecc,
           const double &inclx,
           const double &lon_anodex,
           const double &arg_perix,
           const double &true_anomx);

/**
     * @brief Projects initial state through 'tgo' to final state along a Keplerian
     *      trajectory
     *      Based on Ray Morth, unpublished utility
     *
     * @return kepler_flan = 0: good Kepler projection;
     *                     = 1: bad (# of iterations>20, or neg. sqrt), no new proj cal,
     *                          use prev value;  - ND
     * @param[out] SPII = projected inertial position after tgo - m
     * @param[out] VPII = projected inertial velocity after tgo - m/s
     *
     * @param[in] SBII = current inertial position - m
     * @param[in] VBII = current inertial velocity - m/s
     * @param[in] tgo = time-to-go to projected point - sec
     *
     * @author 040319 Created from FORTRAN by Peter H Zipfel
     */
int kepler(arma::vec3 &SPII,
           arma::vec3 &VPII,
           arma::vec3 SBII,
           arma::vec3 VBII,
           const double &tgo);

/**
     * @brief Projects initial state through 'tgo' to final state along a Keplerian
     *      trajectory
     *      Based on: Bate, Mueller, White, "Fundamentals of Astrodynamics", Dover 1971
     *
     * @return iter_flan = 0: # of iterations < 20;
     *                   = 1: # of iterations > 20;  - ND
     *
     * @param[out] SPII = projected inertial position after tgo - m
     * @param[out] VPII = projected inertial velocity after tgo - m/s
     *
     * @param[in] SBII = current inertial position - m
     * @param[in] VBII = current inertial velocity - m/s
     * @param[in] tgo = time-to-go to projected point - sec
     *
     * @author 040318 Created from ASTRO_KEP by Peter H Zipfel
     */
int kepler1(arma::vec3 &SPII,
            arma::vec3 &VPII,
            arma::vec3 SBII,
            arma::vec3 VBII,
            const double &tgo);

/**
     * @brief Calculates utility functions c(z) and s(z) for kepler(...)
     *      Reference: Bate, Mueller, White, "Fundamentals of Astrodynamics", Dover 1971,
     *      p.196
     * 
     * @param[out] c = c(z) utility function
     * @param[out] s = s(z) utility function
     *
     * @param[in] z = z-variable
     * 
     * @author 040318 Created from ASTRO_UCS by Peter H Zipfel
     */
std::tuple<double, double> kepler1_ucs(const double &z);

/**
     * @brief Calculates the orbital elements from inertial displacement and velocity
     *      Reference: Bate et al. "Fundamentals of Astrodynamics", Dover 1971, p.58
     * 
     * @return cadorbin_flag = 0 ok
     *                         1 'true_anomx' not calculated, because of circular orbit
     *                         2 'semi' not calculated, because parabolic orbit
     *                         3 'lon_anodex' not calculated, because equatorial orbit
     *                         13 'arg_perix' not calculated, because equatorialand/or
     *                            circular orbit
     * @param[out] semi = semi-major axis of orbital ellipsoid - m
     * @param[out] ecc = eccentricity of elliptical orbit - ND
     * @param[out] inclx = inclination of orbital wrt equatorial plane - deg
     * @param[out] lon_anodex = celestial longitude of the ascending node - deg
     * @param[out] arg_perix = argument of periapsis (ascending node to periapsis) - deg
     * @param[out] true_anomx = true anomaly (periapsis to satellite) - deg
     * 
     * @param[in] SBII = Inertial position - m
     * @param[in] VBII = Inertial velocity - m/s
     * 
     * @author 040510 Created by Peter H Zipfel
     * @author 170121 Create Armadillo Version by soncyang
     */
int orb_in(double &semi,
           double &ecc,
           double &inclx,
           double &lon_anodex,
           double &arg_perix,
           double &true_anomx,
           arma::vec3 SBII,
           arma::vec3 VBII);

/**
     * @brief Returns the T.M. of geodetic wrt inertial coordinates
     *        using the WGS 84 reference ellipsoid
     *
     * @return TDI(3x3) = T.M.of geosetic wrt inertial coord - ND
     *
     * @param[out] lon = geodetic longitude - rad
     * @param[out] lat = geodetic latitude - rad
     * @param[out] alt = altitude above ellipsoid - m
     *
     * @author 030424 Created by Peter H Zipfel
     * @author 170121 Create Armadillo Version by sonicyang
     */
arma::mat33 tdi84(const double &lon,
                  const double &lat,
                  const double &alt,
                  arma::mat33 TEI);

arma::mat33 tde84(const double &lon,
                  const double &lat,
                  const double &alt);

/**
     * @brief Returns the T.M. of earth wrt inertial coordinates
     *
     * @return TEI = T.M. of Earthy wrt inertial coordinates
     *
     * @param[in] time = since start of simulation - s
     *
     * @author 010628 Created by Peter H Zipfel
     */
arma::mat33 tei(const double &time);

/**
     * @return TGE = the T.M. of geographic wrt earth coordinates, TGE
     *               spherical Earth only
     *
     * @param[in] lon = geographic longitude - rad
     * @param[in] lat = geographic latitude - rad
     *
     * @author 010628 Created by Peter H Zipfel
     */
arma::mat33 tge(const double &lon, const double &lat);

/**
     * @brief Returns the T.M. of geographic (geocentric) wrt inertial
     *        using the WGS 84 reference ellipsoid
     *        Reference: Britting,K.R."Inertial Navigation Systems Analysis",
     *        pp.45-49, Wiley, 1971
     *
     * @return TGI(3x3) = T.M.of geographic wrt inertial coord - ND
     * 
     * @param[in] lon = geodetic longitude - rad
     * @param[in] lat = geodetic latitude - rad
     * @param[in] alt = altitude above ellipsoid - m
     *
     * @author 030414 Created from FORTRAN by Peter H Zipfel
     * @author 170121 Create Armadillo Version by soncyang
     */
arma::mat33 tgi84(const double &lon,
                  const double &lat,
                  const double &alt,
                  arma::mat33 TEI);

/**
     * @brief Returns the transformation matrix of inertial wrt perifocal coordinates
     *
     * @return TIP = TM of inertial wrt perifocal
     *
     * @param[in] incl = inclination of orbital wrt equatorial plane - rad
     * @param[in] lon_anode = celestial longitude of the ascending node - rad
     * @param[in] arg_peri = argument of periapsis (ascending node to periapsis) - rad
     *
     * @author 040510 Created by Peter H Zipfel
     */
arma::mat33 tip(const double &incl,
                const double &lon_anode,
                const double &arg_peri);
}  // namespace cad
#endif  // __CADAC_UTIL_HH__
