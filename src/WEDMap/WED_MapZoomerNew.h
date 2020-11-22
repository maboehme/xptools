/*
 * Copyright (c) 2004, Laminar Research.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
#ifndef WED_MAPZOOMERNEW_H
#define WED_MAPZOOMERNEW_H

#include "GUI_ScrollerPane.h"
#include "CompGeomDefs2.h"
#include "CompGeomDefs3.h"

/*
	WED_MapProjection

	A map projection that converts lat/lon coordinates to XY coordinates (i.e.
	world coordinates as used in OpenGL drawing) and back.

	The specific projection that is used is an equirectangular projection
	(https://en.wikipedia.org/wiki/Equirectangular_projection). Among other
	things, this guarantees that the coordinate axes can be converted
	independently of each other.
*/

class WED_MapProjection
{
public:
	WED_MapProjection();

	// Sets the point in lon/lat coordinates that corresponds to the origin in
	// XY coordinates.
	void SetOriginLL(const Point2& ll);

	// Sets the standard parallel. This is the latitude at which the scale of the
	// projection is true, i.e. where the projection does not introduce
	// distortion. Note that this does not need to be the same latitude as the
	// origin set with SetOriginLL().
	void SetStandardParallel(double lat);

	// Sets the number of units in XY space that correspond to one degree of
	// latitude or one meter.
	void SetXYUnitsPerDegLat(double unitsPerDeg);
	void SetXYUnitsPerMeter(double unitsPerMeter);

	// Returns the number of units in XY space that correspond to one meter.
	double XYUnitsPerMeter() const;

	// Converts between lat/lon and XY along a single axis.
	double XToLon(double x) const;
	double YToLat(double y) const;
	double LonToX(double lon) const;
	double LatToY(double lat) const;

	// Converts a single point between lat/lon and XY.
	Point2 XYToLL(const Point2& p) const;
	Point2 LLToXY(const Point2& p) const;

	// Converts multiple points between lat/lon and XY.
	void XYToLLv(Point2 * dst, const Point2 * src, int n) const;
	void LLToXYv(Point2 * dst, const Point2 * src, int n) const;

private:
	double mOriginLat;
	double mOriginLon;
	double mUnitsPerDegLat;
	double mStandardParallelCos;
};

/*
	WED_Camera

	A camera combines a position and orientation in space with an orthogonal or
	perspective projection.
*/

class WED_Camera {
public:
	// Returns whether a point or bounding box (in XYZ coordinates) is (partially) visible.
	virtual bool PointVisible(const Point3& point) const = 0;
	virtual bool BboxVisible(const Bbox3& bbox) const = 0;

	// Returns the distance of the point from the camera's center of projection (if the camera uses a
	// perspective projection) or 0.0 if the camera uses an orthogonal projection.
	virtual double PointDistance(const Point3& point) const = 0;

	// Returns the size in pixels that a feature of the given size at the given distance from the camera
	// will have on screen.
	virtual double PixelSize(double zCamera, double featureSize) const = 0;

	// Returns the size in pixels that an object with the given bounding box (in XYZ coordinates)
	// will have on screen.
	virtual double PixelSize(const Bbox3& bbox) const = 0;

	// Returns the maximum size in pixels that a feature of the given size (in XYZ coordinates) somewhere
	// within the given bounding box (in XYZ coordinates) will have on screen.
	virtual double PixelSize(const Bbox3& bbox, double featureSize) const = 0;

	// Returns the size in pixels that an object at the given position (in XYZ coordinates) and
	// with the given diameter (in XYZ coordinates) will have on screen.
	virtual double PixelSize(const Point3& position, double diameter) const = 0;

	virtual void PushMatrix() = 0;
	virtual void PopMatrix() = 0;
	virtual void Translate(const Vector3& v) = 0;
	virtual void Scale(double sx, double sy, double sz) = 0;
	virtual void Rotate(double deg, const Vector3& axis) = 0;
};

// TODO: The division of responsibilities we want is that the camera deals only in XYZ
// coordinates and the MapProjection is responsible for the conversion.
class WED_PerspectiveCamera : public WED_Camera {
public:
	WED_PerspectiveCamera(double nearClip, double farClip)
		: mNearClip(nearClip), mFarClip(farClip), mWidth(mNearClip), mHeight(mNearClip),
		mViewportWidth(1024), mViewportHeight(1024),
		mFrustumPlanesDirty(true)
	{
		mXform.position = Point3(0, 0, 0);
		mXform.forward = Vector3(0, 1, 0);
		mXform.right = Vector3(1, 0, 0);
		mXform.up = Vector3(0, 0, 1);
	}

	void MoveTo(const Point3& position)
	{
		mXform.position = position;

		mFrustumPlanesDirty = true;
	}

	void SetForward(const Vector3& forward)
	{
		mXform.forward = forward;
		mXform.forward.normalize();
		// TODO: Need to do something sensible when mForward points essentially straight up.
		mXform.right = mXform.forward.cross(Vector3(0, 0, 1));
		mXform.right.normalize();
		mXform.up = mXform.right.cross(mXform.forward);
		// Strictly, this normalization isn't needed. Leave it in anyway?
		mXform.up.normalize();

		mFrustumPlanesDirty = true;
	}

	void SetFOV(double horizontalFovDeg, double viewportWidth, double viewportHeight);

	void ApplyProjectionMatrix();

	void ApplyModelViewMatrix();

	const Point3& Position() const
	{
		return mXform.position;
	}

	const Vector3& Forward() const
	{
		return mXform.forward;
	}

	const Vector3& Up() const
	{
		return mXform.up;
	}

	const Vector3& Right() const
	{
		return mXform.right;
	}

	bool PointVisible(const Point3& point) const override
	{
		UpdateFrustumPlanes();

		for (const auto& plane : mFrustumPlanes)
			if (!plane.on_normal_side(point))
				return false;

		return true;
	}

	bool BboxVisible(const Bbox3& bbox) const override
	{
		UpdateFrustumPlanes();

		// For a bounding box to be invisible, all of its corners must be on the
		// non-visible side of at least one of the bounding planes.
		// This is a pretty simple test, but it's pretty expensive. Look at ways
		// to make this less expensive.

		for (const auto& plane : mFrustumPlanes) {
			bool visible = false;
			bbox.for_each_corner([&plane, &visible](const Point3& corner)
			{
				if (plane.on_normal_side(corner))
				{
					visible = true;
					return false;
				}
				return true;
			});
			if (!visible)
				return false;
		}

		return true;
	}

	double PointDistance(const Point3& point) const override
	{
		return mXform.forward.dot(point - mXform.position);
	}

	double PixelSize(double zCamera, double featureSize) const override
	{
		// TODO: Precompute some stuff so we only have a single value that we
		// divide by zCamera. Even better, turn this into a "should this be
		// visible" function that takes a minimum pixel size. This helps us
		// avoid the division.
		return featureSize / zCamera * mNearClip / mWidth * mViewportWidth;
	}

	double PixelSize(const Bbox3& bbox) const override
	{
		// TODO: This is a pretty crude approximation -- maybe revisit.
		double diameter = sqrt((bbox.p2 - bbox.p1).squared_length());
		return PixelSize(bbox, diameter);
	}

	double PixelSize(const Bbox3& bbox, double featureSize) const override
	{
		double zCameraMin = mFarClip;
		bbox.for_each_corner([this, &zCameraMin](const Point3 &corner)
		{
			double zCamera = PointDistance(corner);
			if (zCamera < zCameraMin)
				zCameraMin = zCamera;
			return true;
		});

		if (zCameraMin < mNearClip)
			zCameraMin = mNearClip;

		return PixelSize(zCameraMin, featureSize);
	}

	double PixelSize(const Point3& position, double diameter) const override
	{
		// z coordinate of the point in camera coordinates.
		double zCamera = PointDistance(position);

		// TODO: Precompute some stuff so we only have a single value that we
		// divide by zCamera. Even better, turn this into a "should this be
		// visible" function that takes a minimum pixel size. This helps us
		// avoid the division.
		return PixelSize(zCamera, diameter);
	}

	void PushMatrix() override;
	void PopMatrix() override;
	void Translate(const Vector3& v) override;
	void Scale(double sx, double sy, double sz) override;
	void Rotate(double deg, const Vector3& axis) override;

private:
	void UpdateFrustumPlanes() const
	{
		if (!mFrustumPlanesDirty) return;

		// Normal points towards inside of frustum.
		mFrustumPlanes[0] = Plane3(Position() + Forward() * mNearClip, Forward());
		mFrustumPlanes[1] = Plane3(Position() + Forward() * mFarClip, -Forward());
		mFrustumPlanes[2] = Plane3(Position(), -Right() * mNearClip + Forward() * (mWidth / 2));
		mFrustumPlanes[3] = Plane3(Position(),  Right() * mNearClip + Forward() * (mWidth / 2));
		mFrustumPlanes[4] = Plane3(Position(), -Up() * mNearClip + Forward() * (mHeight / 2));
		mFrustumPlanes[5] = Plane3(Position(),  Up() * mNearClip + Forward() * (mHeight / 2));

		mFrustumPlanesDirty = false;
	}

	bool ModelViewMatrixConsistent();

	struct Transformation
	{
		Point3 position;
		Vector3 forward, right, up;
	};

	static constexpr int kNumFrustumPlanes = 6;

	double mNearClip, mFarClip, mWidth, mHeight;
	double mViewportWidth, mViewportHeight;
	Transformation mXform;
	std::vector<Transformation> mXformStack;

	mutable bool mFrustumPlanesDirty;
	mutable Plane3 mFrustumPlanes[kNumFrustumPlanes];
};

/*

	The map zoomer maintains a relationship between logical and screen coordinates
	AND maintains a "windowing" system.  Thus it does scrolling and viewing at the same time.

	It does this with two rectangles:

	- Visible bounds - what we can see.
	- Total bounds - the entire image.

	It does this with two coordinate systems:

	- Pixels - screen drawing units.
	- Logical - whatever units our map is in (degrees lat/lon for WED).

	Thus we have the pixel visible bounds, logical visible bounds, and logical total bounds.

*/

class	WED_MapZoomerNew : public GUI_ScrollerPaneContent, public WED_Camera {
public:

					 WED_MapZoomerNew();
	virtual			~WED_MapZoomerNew();
	// The map zoomer converts lat/lon coordinates to pixel coordinates.
	// This API is called by just about anything that needs to do coordinate
	// conversion.

			double	XPixelToLon(double) const;
			double	YPixelToLat(double) const;
			double	LonToXPixel(double) const;
			double	LatToYPixel(double) const;

			Point2	PixelToLL(const Point2& p) const;
			Point2	LLToPixel(const Point2& p) const;

			void	PixelToLLv(Point2 * dst, const Point2 * src, int n) const;
			void	LLToPixelv(Point2 * dst, const Point2 * src, int n) const;

			double	GetPPM(void) const;
			double	GetClickRadius(double pixels) const;
			long long	CacheKey(void) const { return mCacheKey; }

			const WED_MapProjection& Projection() const { return mProjection; }

	// This API is called by the map class to set up and modify the zoomer

	// Overall setup

			void	SetMapLogicalBounds(			// Define the max scrollable map positions.
							double	inWest,
							double	inSouth,
							double	inEast,
							double	inNorth);

			void	GetPixelBounds(					// Get the area on the screen the user
							double& outLeft,			// can see.
							double&	outBottom,
							double&	outRight,
							double&	outTop);
			void	GetMapVisibleBounds(			// Get the amount of the map visible in
							double&	outWest,		// this screen area.
							double&	outSouth,
							double&	outEast,
							double&	outNorth);
			void	GetMapLogicalBounds(			// Defoute the max scrollable map positions.
							double&	outWest,
							double&	outSouth,
							double&	outEast,
							double&	outNorth);


	// Scrolling operations
			void	ZoomShowAll(void);				// Zoom out to reveal the whole map
			void	ZoomShowArea(
							double	inWest,
							double	inSouth,
							double	inEast,
							double	inNorth);
			void	PanPixels(						// Pan so that the logical pixel under p1
							double	x1,				// is now visible under p2
							double	y1,
							double	x2,
							double	y2);
			void	ZoomAround(						// Zoom in and out keeping one pixel constant
							double	zoomFactor,
							double	centerXPixel,
							double	centerYPixel);
			void	ScrollReveal(
							double	inLon,
							double	inLat);
			void	ScrollReveal(
							double	inWest,
							double	inSouth,
							double	inEast,
							double	inNorth);


	virtual	void	GetScrollBounds(float outTotalBounds[4], float outVisibleBounds[4]);
	virtual	void	ScrollH(float xOffset);
	virtual	void	ScrollV(float yOffset);

	bool PointVisible(const Point3& point) const override;
	bool BboxVisible(const Bbox3& bbox) const override;
	double PointDistance(const Point3& point) const override
	{
		return 0.0;
	}
	double PixelSize(double zCamera, double featureSize) const override;
	double PixelSize(const Bbox3& bbox) const override;
	double PixelSize(const Bbox3& bbox, double featureSize) const override;
	double PixelSize(const Point3& position, double diameter) const override;
	void PushMatrix() override;
	void PopMatrix() override;
	void Translate(const Vector3& v) override;
	void Scale(double sx, double sy, double sz) override;
	void Rotate(double deg, const Vector3& axis) override;

// MBHACK
// protected:
			void	SetPixelBounds(					// Set the area on the screen the user
							double 	inLeft,			// can see.
							double	inBottom,
							double	inRight,
							double	inTop);


private:

			void	RecalcAspectRatio(void);
			void	UpdateProjection();

	double	mPixels[4];
	double	mLogicalBounds[4];
	double	mLatCenter;
	double	mLonCenter;
	double	mLonCenterCOS;
	long long mCacheKey;
	WED_MapProjection mProjection;
protected:
	double	mPixel2DegLat;

};

#endif
