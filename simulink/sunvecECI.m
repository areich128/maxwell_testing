function [sun_vec_eci, R, declination, right_ascension] = sunvecECI(utc_time)
% sun_vector_eci compute sun vector in Earth-Centered Inertial coordinates
% 
% inputs:
%       time_input - UTC
% outputs:
%       sun_vec_eci = 3x1 unit vector from Earth to Sun in ECI
%       R = Sun-Earth distance in AU
%       declination = Sun's angular distance north or south of EArth's
%       equator (degrees)
%       right_ascension = Sun's angular positoin measured eastward along
%       celestial equator (degrees)

% Uses USNO algorithm: https://aa.usno.navy.mil/faq/sun_approx
%% Time
% Calculate Julian Date
% jd = juliandate(year, month, day, hour, minute, second)
jd = juliandate(utc_time);

% Set time relative to J2000.0
D = jd - 2451545.0;

%% Solar Position Calculations
% All constants are in degrees

% Mean longitude of the Sun
q = 280.459 + (36000.77005361/36525)*D;

% Mean anomaly of the Sun
g = 357.529 + (35999.05034/36525)*D;

% Geocentric apparent ecliptic longitude of the Sun (adjusted for
% aberration)
L = q + 1.914666471*sind(g) + 0.018994643*sind(2*g);

% The Sun's ecliptic latitude, b, is approximated by b=0

% The distance of the Sun from the Earth, R, in astronomical units (AU)
R = 1.000140612 - 0.016708617*cosd(g) - 0.000139589*cosd(2*g);

%% Sun's Right Ascension and Declination

e = 23.43929 - (46.8093/3600)*(D/36525); % mean obliquity of the ecliptic (degrees)
right_ascension = atan2d(cosd(e)*sind(L), cosd(L));

if right_ascension < 0
    right_ascension = right_ascension + 360;
end

declination = asind(sind(e)*sind(L));

%% ECI Vector

% spherical to cartesian coordinate conversion in celestial sphere
x_eci = cosd(declination) * cosd(right_ascension);
y_eci = cosd(declination) * sind(right_ascension);
z_eci = sind(declination);

sun_vec_eci = [x_eci; y_eci; z_eci];

end