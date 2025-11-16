
#include "MyConvexHull.h"
using namespace ljgeo;

// Compute convex hull of given 3D points (Z is ignored).
BOOL ConvexHullPoints(AcGePoint3dArray& pileCenters)
{
    SimpleHull hull;

    // Feed all points into the hull (use x, y only).
    int i;
    for (i = 0; i < pileCenters.length(); ++i)
        hull.addPoint(pileCenters[i].x, pileCenters[i].y);

    hull.solve();

    int hullCount = (int)hull.getHullCount();
    if (hullCount <= 0)
        return FALSE;

    AcGePoint3dArray result;
    for (i = 0; i < hullCount; ++i)
    {
        Point2D p = hull.getHullPoint(i);
        result.append(AcGePoint3d(p.x, p.y, 0.0)); // keep Z = 0
    }

    pileCenters = result;
    return TRUE;
}

// test: build convex hull from selected polylines
void test_convex_hull()
{
    ads_printf(_T("\\nSelect polylines for convex hull <Exit>: "));
    ads_name ssname;
    int rc = ads_ssget(NULL, NULL, NULL, NULL, ssname);
    if (rc != RTNORM)
        return;

    AcGePoint3dArray pileCenters;

    Adesk::Int32 sslen = 0;
    if (ads_sslength(ssname, &sslen) != RTNORM || sslen <= 0)
    {
        ads_ssfree(ssname);
        return;
    }

    // Collect vertices from selected polylines.
    for (int i = 0; i < sslen; ++i)
    {
        ads_name ename;
        if (ads_ssname(ssname, i, ename) != RTNORM)
            continue;

        AcDbObjectId id;
        if (acdbGetObjectId(id, ename) != Acad::eOk)
            continue;

        AcDbEntity* pEnt = NULL;
        if (acdbOpenObject(pEnt, id, AcDb::kForRead) != Acad::eOk)
            continue;

        if (pEnt->isKindOf(AcDbPolyline::desc()))
        {
            AcDbPolyline* pPoly = AcDbPolyline::cast(pEnt);
            int vcount = pPoly->numVerts();

            for (int j = 0; j < vcount; ++j)
            {
                AcGePoint2d pt2d;
                pPoly->getPointAt(j, pt2d);
                pileCenters.append(My2d23d(pt2d)); // store as 3D point (Z=0)
            }
        }
		else if (pEnt->isKindOf(AcDbLine::desc()))
		{
			// Use both endpoints of the line.
			AcDbLine* pLine = AcDbLine::cast(pEnt);
			AcGePoint3d spt = pLine->startPoint();
			AcGePoint3d ept = pLine->endPoint();

			pileCenters.append(spt);
			pileCenters.append(ept);
		}
		else if (pEnt->isKindOf(AcDbPoint::desc()))
		{
			// Use the position of the dbpoint.
			AcDbPoint* pPt = AcDbPoint::cast(pEnt);
			pileCenters.append(pPt->position());
		}
        pEnt->close();
    }

    ads_ssfree(ssname);

    // Build convex hull on collected points.
    if (!ConvexHullPoints(pileCenters))
        return;

    // Convert back to 2D and draw polyline on layer "0".
    AcGePoint2dArray pts;
    for (int i = 0; i < pileCenters.length(); ++i)
        pts.append(My3d22d(pileCenters[i]));

    Poly(pts, _T("0"));
}
