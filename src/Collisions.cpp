#include "Collisions.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace {

double boxDistanceSq(const AABB &a, const AABB &b) {
    const double x = std::max({0.0, a.min.x - b.max.x, b.min.x - a.max.x});
    const double y = std::max({0.0, a.min.y - b.max.y, b.min.y - a.max.y});
    const double z = std::max({0.0, a.min.z - b.max.z, b.min.z - a.max.z});
    return x * x + y * y + z * z;
}

bool overlapOnAxis(const Vec3 &axis, const Vec3 p[3], const Vec3 q[3], double epsSq) {
    if (axis.lengthSq() <= epsSq)
        return true;
    double minA = dot(axis, p[0]), maxA = minA;
    double minB = dot(axis, q[0]), maxB = minB;
    for (int i = 1; i < 3; ++i) {
        const double a = dot(axis, p[i]), b = dot(axis, q[i]);
        minA = std::min(minA, a);
        maxA = std::max(maxA, a);
        minB = std::min(minB, b);
        maxB = std::max(maxB, b);
    }
    return !(maxA < minB || maxB < minA);
}

bool trianglesIntersect(const Vec3 p[3], const Vec3 q[3]) {
    const Vec3 ea[3] = {p[1] - p[0], p[2] - p[1], p[0] - p[2]};
    const Vec3 eb[3] = {q[1] - q[0], q[2] - q[1], q[0] - q[2]};
    const Vec3 na = cross(ea[0], ea[1]), nb = cross(eb[0], eb[1]);
    if (!overlapOnAxis(na, p, q, 0) || !overlapOnAxis(nb, p, q, 0))
        return false;
    for (const auto &a : ea)
        for (const auto &b : eb)
            if (!overlapOnAxis(cross(a, b), p, q, 1e-24 * a.lengthSq() * b.lengthSq()))
                return false;
    // For coplanar triangles all the above axes are normal to the plane.
    // In-plane axes are necessary to distinguish disjoint triangles.
    if (cross(na, nb).lengthSq() <= 1e-24 * na.lengthSq() * nb.lengthSq()) {
        for (const auto &e : ea)
            if (!overlapOnAxis(cross(na, e), p, q, 0))
                return false;
        for (const auto &e : eb)
            if (!overlapOnAxis(cross(nb, e), p, q, 0))
                return false;
    }
    return true;
}

double pointSegmentSq(const Vec3 &p, const Vec3 &a, const Vec3 &b) {
    const Vec3 e = b - a;
    const double t = e.lengthSq() > 0 ? std::clamp(dot(p - a, e) / e.lengthSq(), 0.0, 1.0) : 0;
    return (p - (a + e * t)).lengthSq();
}

double pointTriangleSq(const Vec3 &p, const Vec3 t[3]) {
    const Vec3 a = t[0], b = t[1], c = t[2], ab = b - a, ac = c - a, ap = p - a;
    if (cross(ab, ac).lengthSq() <= 1e-28 * ab.lengthSq() * ac.lengthSq())
        return std::min({pointSegmentSq(p, a, b), pointSegmentSq(p, b, c), pointSegmentSq(p, c, a)});
    const double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0)
        return ap.lengthSq();
    const Vec3 bp = p - b;
    const double d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3)
        return bp.lengthSq();
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0)
        return (p - (a + ab * (d1 / (d1 - d3)))).lengthSq();
    const Vec3 cp = p - c;
    const double d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6)
        return cp.lengthSq();
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0)
        return (p - (a + ac * (d2 / (d2 - d6)))).lengthSq();
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 >= d3 && d5 >= d6)
        return (p - (b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6))))).lengthSq();
    const double den = 1 / (va + vb + vc);
    return (p - (a + ab * (vb * den) + ac * (vc * den))).lengthSq();
}

double segmentDistanceSq(const Vec3 &p, const Vec3 &q, const Vec3 &r, const Vec3 &s) {
    const Vec3 a = q - p, b = s - r, d = p - r;
    const double aa = dot(a, a), bb = dot(b, b), ab = dot(a, b), ad = dot(a, d), bd = dot(b, d);
    if (aa == 0)
        return pointSegmentSq(p, r, s);
    if (bb == 0)
        return pointSegmentSq(r, p, q);
    const double den = aa * bb - ab * ab;
    double u = den > 1e-15 * aa * bb ? std::clamp((ab * bd - ad * bb) / den, 0.0, 1.0) : 0;
    double v = (ab * u + bd) / bb;
    if (v < 0) {
        v = 0;
        u = std::clamp(-ad / aa, 0.0, 1.0);
    } else if (v > 1) {
        v = 1;
        u = std::clamp((ab - ad) / aa, 0.0, 1.0);
    }
    return (d + a * u - b * v).lengthSq();
}

bool trianglesTooClose(const Vec3 p[3], const Vec3 q[3], double gapSq) {
    if (trianglesIntersect(p, q))
        return true;
    if (gapSq <= 0)
        return false;
    for (int i = 0; i < 3; ++i) {
        if (pointTriangleSq(p[i], q) < gapSq || pointTriangleSq(q[i], p) < gapSq)
            return true;
        for (int j = 0; j < 3; ++j)
            if (segmentDistanceSq(p[i], p[(i + 1) % 3], q[j], q[(j + 1) % 3]) < gapSq)
                return true;
    }
    return false;
}

bool surfaceConflict(const BVHNode &a, const Mesh &ma, const BVHNode &b, const Mesh &mb, const Mat3 &r, const Vec3 &t,
                     double gapSq) {
    const AABB bb = transformAABB(b.box, r, t);
    if (boxDistanceSq(a.box, bb) > gapSq)
        return false;
    if (a.isLeaf() && b.isLeaf()) {
        for (int ia : a.tri_indices)
            for (int ib : b.tri_indices) {
                Vec3 p[3], q[3];
                for (int i = 0; i < 3; ++i) {
                    p[i] = ma.vertices[ma.indices[ia * 3 + i]];
                    q[i] = r.transform(mb.vertices[mb.indices[ib * 3 + i]]) + t;
                }
                if (trianglesTooClose(p, q, gapSq))
                    return true;
            }
        return false;
    }
    if (b.isLeaf() || (!a.isLeaf() && a.box.volume() > bb.volume()))
        return surfaceConflict(*a.left, ma, b, mb, r, t, gapSq) || surfaceConflict(*a.right, ma, b, mb, r, t, gapSq);
    return surfaceConflict(a, ma, *b.left, mb, r, t, gapSq) || surfaceConflict(a, ma, *b.right, mb, r, t, gapSq);
}

bool rayBox(const Vec3 &o, const Vec3 &d, const AABB &b) {
    double near = 0, far = 1e300;
    const double oo[] = {o.x, o.y, o.z}, dd[] = {d.x, d.y, d.z};
    const double lo[] = {b.min.x, b.min.y, b.min.z}, hi[] = {b.max.x, b.max.y, b.max.z};
    for (int i = 0; i < 3; ++i) {
        if (std::abs(dd[i]) < 1e-15) {
            if (oo[i] < lo[i] || oo[i] > hi[i])
                return false;
            continue;
        }
        double x = (lo[i] - oo[i]) / dd[i], y = (hi[i] - oo[i]) / dd[i];
        if (x > y)
            std::swap(x, y);
        near = std::max(near, x);
        far = std::min(far, y);
        if (near > far)
            return false;
    }
    return true;
}

void rayHits(const BVHNode &node, const Mesh &mesh, const Vec3 &o, const Vec3 &d, double eps,
             std::vector<double> &hits) {
    if (!rayBox(o, d, node.box))
        return;
    if (!node.isLeaf()) {
        rayHits(*node.left, mesh, o, d, eps, hits);
        rayHits(*node.right, mesh, o, d, eps, hits);
        return;
    }
    for (int i : node.tri_indices) {
        const Vec3 a = mesh.vertices[mesh.indices[i * 3]], b = mesh.vertices[mesh.indices[i * 3 + 1]],
                   c = mesh.vertices[mesh.indices[i * 3 + 2]];
        const Vec3 e = b - a, f = c - a, h = cross(d, f), s = o - a;
        const double det = dot(e, h);
        if (std::abs(det) < 1e-14 * e.length() * f.length())
            continue;
        const double u = dot(s, h) / det;
        const Vec3 q = cross(s, e);
        const double v = dot(d, q) / det, t = dot(f, q) / det;
        if (u >= -1e-12 && v >= -1e-12 && u + v <= 1 + 1e-12 && t > eps)
            hits.push_back(t);
    }
}

bool inside(const Mesh &mesh, const Vec3 &point, double eps) {
    if (!mesh.bvh_root->box.contains(point))
        return false;
    // Deduplicate hits on shared edges: two adjacent triangles represent one
    // crossing. Majority voting avoids treating a tangent ray as containment.
    const Vec3 directions[] = {
        {0.7385489, 0.4431293, 0.5076391}, {-0.5144958, 0.6859943, -0.5144958}, {0.312317, -0.821143, 0.477719}};
    int votes = 0;
    for (const Vec3 &d : directions) {
        std::vector<double> hits;
        rayHits(*mesh.bvh_root, mesh, point, d, eps, hits);
        std::sort(hits.begin(), hits.end());
        int unique = 0;
        double previous = -1e300;
        for (double t : hits)
            if (t - previous > eps) {
                ++unique;
                previous = t;
            }
        votes += unique % 2;
    }
    return votes >= 2;
}

std::unique_ptr<BVHNode> build(const Mesh &mesh, const std::vector<Vec3> &centers, std::vector<int> &tris, size_t begin,
                               size_t end) {
    auto node = std::make_unique<BVHNode>();
    for (size_t i = begin; i < end; ++i)
        for (int k = 0; k < 3; ++k)
            node->box.expand(mesh.vertices[mesh.indices[tris[i] * 3 + k]]);
    if (end - begin <= 4) {
        node->tri_indices.assign(tris.begin() + begin, tris.begin() + end);
        return node;
    }
    const Vec3 ext = node->box.max - node->box.min;
    const int axis = ext.y >= ext.x && ext.y >= ext.z ? 1 : (ext.z >= ext.x ? 2 : 0);
    const size_t mid = (begin + end) / 2;
    auto coord = [axis](const Vec3 &v) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; };
    std::nth_element(tris.begin() + begin, tris.begin() + mid, tris.begin() + end,
                     [&](int a, int b) { return coord(centers[a]) < coord(centers[b]); });
    node->left = build(mesh, centers, tris, begin, mid);
    node->right = build(mesh, centers, tris, mid, end);
    return node;
}
} // namespace

void buildMeshAcceleration(Mesh &mesh) {
    if (mesh.indices.empty() || mesh.indices.size() % 3)
        throw std::runtime_error("Mesh must contain triangles");
    std::vector<int> parents(mesh.vertices.size());
    std::iota(parents.begin(), parents.end(), 0);
    auto root = [&](int v) {
        while (parents[v] != v) {
            parents[v] = parents[parents[v]];
            v = parents[v];
        }
        return v;
    };
    std::vector<Vec3> centers(mesh.indices.size() / 3);
    std::vector<int> tris(centers.size());
    std::iota(tris.begin(), tris.end(), 0);
    for (size_t i = 0; i < centers.size(); ++i) {
        const int a = mesh.indices[3 * i], b = mesh.indices[3 * i + 1], c = mesh.indices[3 * i + 2];
        parents[root(b)] = root(a);
        parents[root(c)] = root(a);
        centers[i] = (mesh.vertices[a] + mesh.vertices[b] + mesh.vertices[c]) * (1.0 / 3);
    }
    mesh.componentVertices.clear();
    std::vector<bool> seen(parents.size(), false);
    for (int v : mesh.indices)
        if (!seen[root(v)]) {
            seen[root(v)] = true;
            mesh.componentVertices.push_back(v);
        }
    mesh.bvh_root = build(mesh, centers, tris, 0, tris.size());
}

AABB transformAABB(const AABB &local, const Mat3 &mat, const Vec3 &pos) {
    AABB out;
    out.min = out.max = pos;
    const double lo[] = {local.min.x, local.min.y, local.min.z}, hi[] = {local.max.x, local.max.y, local.max.z};
    double a[] = {pos.x, pos.y, pos.z}, b[] = {pos.x, pos.y, pos.z};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            const double x = mat.m[i][j] * lo[j], y = mat.m[i][j] * hi[j];
            a[i] += std::min(x, y);
            b[i] += std::max(x, y);
        }
    out.min = {a[0], a[1], a[2]};
    out.max = {b[0], b[1], b[2]};
    return out;
}

AABB computeTightAABB(const Mesh &mesh, const Mat3 &mat, const Vec3 &pos) {
    AABB box;
    for (const Vec3 &v : mesh.vertices)
        box.expand(mat.transform(v) + pos);
    return box;
}

bool meshesConflict(const Mesh &a, const Mat3 &ra, const Vec3 &pa, const AABB &ba, const Mesh &b, const Mat3 &rb,
                    const Vec3 &pb, const AABB &bb, double gap) {
    const double eps = 1e-9 * std::max({(ba.max - ba.min).length(), (bb.max - bb.min).length(), gap, 1e-12});
    // Allow exact wall-to-wall box contact at zero clearance. No solid interiors
    // can overlap when their bounding boxes only touch on an axis.
    if (gap == 0 && ba.volume() > 0 && bb.volume() > 0 &&
        (std::min(ba.max.x, bb.max.x) - std::max(ba.min.x, bb.min.x) <= eps ||
         std::min(ba.max.y, bb.max.y) - std::max(ba.min.y, bb.min.y) <= eps ||
         std::min(ba.max.z, bb.max.z) - std::max(ba.min.z, bb.min.z) <= eps))
        return false;
    const double effectiveGap = std::max(0.0, gap - eps);
    if (boxDistanceSq(ba, bb) > gap * gap)
        return false;
    const Mat3 relative = ra.transposeMultiply(rb);
    const Vec3 translation = ra.transposeTransform(pb - pa);
    if (surfaceConflict(*a.bvh_root, a, *b.bvh_root, b, relative, translation, effectiveGap * effectiveGap))
        return true;
    for (int v : a.componentVertices)
        if (inside(b, rb.transposeTransform(ra.transform(a.vertices[v]) + pa - pb), eps))
            return true;
    for (int v : b.componentVertices)
        if (inside(a, ra.transposeTransform(rb.transform(b.vertices[v]) + pb - pa), eps))
            return true;
    return false;
}
