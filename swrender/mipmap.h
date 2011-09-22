/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2011 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#if !defined(__MIPMAP_H)
#define __MIPMAP_H
#include <opencv/cv.h>
#include <osg/Vec2>
#define MIPMAP_LUTSIZE 128

/** \brief Isotropic/anisotropic EWA mip-map texture map class based on PBRT 
 * \ingroup librender
 */
#define SPECTRUM_SAMPLES  3

struct  Spectrum {
public:
    typedef float Float;

        /**
         * \brief When converting from RGB reflectance values to
         * discretized color spectra, the following `intent' flag
         * can be provided to improve the results of this highly
         * under-constrained problem.
         */
        enum EConversionIntent {
                /// Unitless reflectance data is converted
                EReflectance,

                /// Radiance-valued illumination data is converted
                EIlluminant
        };

        /// Create a new spectral power distribution, but don't initialize the contents
        inline Spectrum() { }

        /// Create a new spectral power distribution with all samples set to the given value
        explicit inline Spectrum(Float v) {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        s[i] = v;
        }

        /// Copy a spectral power distribution
        explicit inline Spectrum(Float spd[SPECTRUM_SAMPLES]) {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        s[i] = spd[i];
        }


        /// Add two spectral power distributions
        inline Spectrum operator+(const Spectrum &spd) const {
                Spectrum value = *this;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        value.s[i] += spd.s[i];
                return value;
        }

        /// Add a spectral power distribution to this instance
        inline Spectrum& operator+=(const Spectrum &spd) {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        s[i] += spd.s[i];
                return *this;
        }

        /// Subtract a spectral power distribution
        inline Spectrum operator-(const Spectrum &spd) const {
                Spectrum value = *this;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        value.s[i] -= spd.s[i];
                return value;
        }

        /// Subtract a spectral power distribution from this instance
        inline Spectrum& operator-=(const Spectrum &spd) {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        s[i] -= spd.s[i];
                return *this;
        }

        /// Multiply by a scalar
        inline Spectrum operator*(Float f) const {
                Spectrum value = *this;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        value.s[i] *= f;
                return value;
        }

        /// Multiply by a scalar
        inline friend Spectrum operator*(Float f, Spectrum &spd) {
                return spd * f;
        }

        /// Multiply by a scalar
        inline Spectrum& operator*=(Float f) {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        s[i] *= f;
                return *this;
        }

        /// Perform a component-wise multiplication by another spectrum
        inline Spectrum operator*(const Spectrum &spd) const {
                Spectrum value = *this;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        value.s[i] *= spd.s[i];
                return value;
        }

        /// Perform a component-wise multiplication by another spectrum
        inline Spectrum& operator*=(const Spectrum &spd) {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        s[i] *= spd.s[i];
                return *this;
        }

        /// Perform a component-wise division by another spectrum
        inline Spectrum& operator/=(const Spectrum &spd) {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        s[i] /= spd.s[i];
                return *this;
        }

        /// Perform a component-wise division by another spectrum
        inline Spectrum operator/(Spectrum spd) const {
                Spectrum value = *this;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        value.s[i] /= spd.s[i];
                return value;
        }

        /// Divide by a scalar
        inline Spectrum operator/(Float f) const {
                Spectrum value = *this;
#ifdef MTS_DEBUG
                if (f == 0)
                        SLog(EWarn, "Spectrum: Division by zero!");
#endif
                Float recip = 1.0f / f;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        value.s[i] *= recip;
                return value;
        }

        /// Equality test
        inline bool operator==(Spectrum spd) const {
                for (int i=0; i<SPECTRUM_SAMPLES; i++) {
                        if (s[i] != spd.s[i])
                                return false;
                }
                return true;
        }

        /// Inequality test
        inline bool operator!=(Spectrum spd) const {
                return !operator==(spd);
        }

        /// Divide by a scalar
        inline friend Spectrum operator/(Float f, Spectrum &spd) {
                return spd / f;
        }

        /// Divide by a scalar
        inline Spectrum& operator/=(Float f) {
#ifdef MTS_DEBUG
                if (f == 0)
                        SLog(EWarn, "Spectrum: Division by zero!");
#endif
                Float recip = 1.0f / f;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        s[i] *= recip;
                return *this;
        }

        /// Check for NaNs
        inline bool isNaN() const {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        if (isnan(s[i]))
                                return true;
                return false;
        }

        /// Returns whether the spectrum only contains valid (non-NaN, nonnegative) samples
        inline bool isValid() const {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        if (isnan(s[i]) || s[i] < 0.0f)
                                return false;
                return true;
        }

        /// Multiply-accumulate operation, adds \a weight * \a spd
        inline void addWeighted(Float weight, const Spectrum &spd) {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        s[i] += weight * spd.s[i];
        }

        /// Return the average over all wavelengths
        inline Float average() const {
                Float result = 0.0f;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        result += s[i];
                return result * (1.0f / SPECTRUM_SAMPLES);
        }

        /// Component-wise square root
        inline Spectrum sqrt() const {
                Spectrum value;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        value.s[i] = std::sqrt(s[i]);
                return value;
        }

        /// Component-wise exponentation
        inline Spectrum exp() const {
                Spectrum value;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        value.s[i] = std::exp(s[i]);
                return value;
        }

        /// Component-wise power
        inline Spectrum pow(Float f) const {
                Spectrum value;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        value.s[i] = std::pow(s[i], f);
                return value;
        }

        /// Clamp negative values
        inline void clampNegative() {
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        s[i] = std::max((Float) 0.0f, s[i]);
        }

        /// Return the highest-valued spectral sample
        inline Float max() const {
                Float result = s[0];
                for (int i=1; i<SPECTRUM_SAMPLES; i++)
                        result = std::max(result, s[i]);
                return result;
        }

        /// Return the lowest-valued spectral sample
        inline Float min() const {
                Float result = s[0];
                for (int i=1; i<SPECTRUM_SAMPLES; i++)
                        result = std::min(result, s[i]);
                return result;
        }

        /// Negate
        inline Spectrum operator-() const {
                Spectrum value;
                for (int i=0; i<SPECTRUM_SAMPLES; i++)
                        value.s[i] = -s[i];
                return value;
        }

        /// Indexing operator
        inline Float &operator[](int entry) {
                return s[entry];
        }

        /// Indexing operator
        inline Float operator[](int entry) const {
                return s[entry];
        }

        /// Check if this spectrum is zero at all wavelengths
        inline bool isZero() const {
                for (int i=0; i<SPECTRUM_SAMPLES; i++) {
                        if (s[i] != 0.0f)
                                return false;
                }
                return true;
        }

        /**
         * \brief Evaluate the SPD for the given wavelength
         * in nanometers.
         */
        Float eval(Float lambda) const;

        /// \brief Return the wavelength range covered by a spectral bin
        static std::pair<Float, Float> getBinCoverage(size_t index);

        /// Return the luminance in candelas.
#if SPECTRUM_SAMPLES == 3
        inline Float getLuminance() const {
                return s[0] * 0.212671f + s[1] * 0.715160f + s[2] * 0.072169f;
        }
#else
        Float getLuminance() const;
#endif

        /// Convert from a spectral power distribution to XYZ tristimulus values
        void toXYZ(Float &x, Float &y, Float &z) const;

        /**
         * \brief Convert XYZ tristimulus into a plausible spectral
         * power distribution
         *
         * The \ref EConversionIntent parameter can be used to provide more
         * information on how to solve this highly under-constrained problem.
         * The default is \ref EReflectance.
         */
        void fromXYZ(Float x, Float y, Float z,
                        EConversionIntent intent = EReflectance);

        /// Convert to linear RGB
        inline void toLinearRGB(Float &r, Float &g, Float &b) const {
                /* Nothing to do -- the renderer is in RGB mode */
                r = s[0]; g = s[1]; b = s[2];
        }

        /// Convert from linear RGB
        inline void fromLinearRGB(Float r, Float g, Float b,
                        EConversionIntent intent = EReflectance /* unused */) {
                /* Nothing to do -- the renderer is in RGB mode */
                s[0] = r; s[1] = g; s[2] = b;
        }


protected:
        Float s[SPECTRUM_SAMPLES];


};
class MIPMap {
public:
    typedef float Float;
    Float lanczosSinc(Float t, Float tau=2.0) const;

	enum EWrapMode {
		ERepeat = 0,
		EBlack,
		EWhite,
		EClamp
	};

	enum EFilterType {
		/// Elliptically weighted average
		EEWA,
		/// Trilinear filtering
		ETrilinear,
		/// No filtering
		ENone
	};

	/**
	 * Construct a new mip-map from the given texture. Does not
	 * need to have a power-of-two size.
	 */
        MIPMap(int width, int height, Spectrum *pixels,
		EFilterType filterType = EEWA, EWrapMode wrapMode = ERepeat,
		Float maxAnisotropy = 8.0f);

	/// Construct a mip map from a HDR bitmap
        static MIPMap* fromBitmap(IplImage *bitmap,
		EFilterType filterType = EEWA, EWrapMode wrapMode = ERepeat,
                Float maxAnisotropy = 8.0f
                );

	/// Do a mip-map lookup at the appropriate level
        Spectrum getValue(Float u, Float v,
		Float dudx, Float dudy, Float dvdx, Float dvdy) const;

	/// Return the number of mip-map levels
	inline int getLevels() const { return m_levels; }

	/// Bilinear interpolation using a triangle filter
	Spectrum triangle(int level, Float x, Float y) const;

	/// Return the width of the represented texture
	inline int getWidth() const { return m_width; }

	/// Return the height of the represented texture
	inline int getHeight() const { return m_height; }

	/// Return a pointer to internal image representation at full resolution
	inline const Spectrum *getImageData() const { return m_pyramid[0]; }
	
	/// Return a pointer to internal image representation at the specified resolution
	inline const Spectrum *getImageData(int level) const { return m_pyramid[level]; }

	/// Return the resolution of the specified level
        inline const osg::Vec2 getLevelResolution(int level) const {
            return osg::Vec2(m_levelWidth[level], m_levelHeight[level]);
	}

	/// Get the component-wise maximum at the zero level
	Spectrum getMaximum() const;

	/// Return a bitmap representation of the full-resolution image
        IplImage *getLDRBitmap() const;

	/// Return a bitmap representation of the full-resolution image (8 bit/color)
//	Bitmap *getLDRBitmap() const;

//	MTS_DECLARE_CLASS()

        /// Look up a texel at the given hierarchy level
        Spectrum getTexel(int level, int x, int y) const;
        ~MIPMap();

protected:
	/// \cond
	struct ResampleWeight {
		int firstTexel;
		Float weight[4];
	};
	/// \endcond

	/// Calculate weights for up-sampling a texture
	ResampleWeight *resampleWeights(int oldRes, int newRes) const;

	/**
	 * Calculate the elliptically weighted average of a sample with
     * differential uv information
	 */
	Spectrum EWA(Float u, Float v, Float dudx, Float dudy, Float dvdx, 
		Float dvdy, int level) const;

	/* Virtual destructor */

private:
	int m_width, m_height;
	int m_levels;
	int *m_levelWidth;
	int *m_levelHeight;
	Spectrum **m_pyramid;
	EFilterType m_filterType;
	EWrapMode m_wrapMode;
	Float *m_weightLut;
	Float m_maxAnisotropy;
};

/// Check if an integer is a power of two (unsigned 32 bit version)
inline bool isPowerOfTwo(uint32_t i) { return (i & (i-1)) == 0; }

/// Check if an integer is a power of two (signed 32 bit version)
inline bool isPowerOfTwo(int32_t i) { return i > 0 && (i & (i-1)) == 0; }

/// Check if an integer is a power of two (64 bit version)
inline bool isPowerOfTwo(uint64_t i) { return (i & (i-1)) == 0; }

/// Check if an integer is a power of two (signed 64 bit version)
inline bool isPowerOfTwo(int64_t i) { return i > 0 && (i & (i-1)) == 0; }


#endif /* __MIPMAP_H */
