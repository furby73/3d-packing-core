#include "Packing.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <random>
#include <stdexcept>

namespace {
using Clock = std::chrono::steady_clock;
struct Oriented {
    Vec3 rotation;
    Mat3 matrix;
    AABB box;
};
struct State {
    std::vector<Placement> placements;
    std::vector<Mat3> matrices;
    std::vector<AABB> boxes;
};

bool fits(const AABB &box, const Vec3 &size) {
    const Vec3 e = box.max - box.min;
    const double eps = 1e-10 * std::max({size.x, size.y, size.z});
    return e.x <= size.x + eps && e.y <= size.y + eps && e.z <= size.z + eps;
}
Oriented orient(const Mesh &mesh, const Vec3 &rotation) {
    Oriented out;
    out.rotation = rotation;
    Transform t;
    t.rot = rotation;
    out.matrix = t.getMatrix();
    out.box = computeTightAABB(mesh, out.matrix, {0, 0, 0});
    return out;
}
void place(State &state, size_t i, size_t bin, const Oriented &o, const Vec3 &minimum) {
    state.placements[i] = {{minimum - o.box.min, o.rotation}, bin};
    state.matrices[i] = o.matrix;
    state.boxes[i].min = minimum;
    state.boxes[i].max = minimum + (o.box.max - o.box.min);
}

bool conflict(const std::vector<Mesh> &meshes, const State &s, size_t i, size_t j, double gap) {
    return s.placements[i].bin == s.placements[j].bin &&
           meshesConflict(meshes[i], s.matrices[i], s.placements[i].transform.pos, s.boxes[i], meshes[j], s.matrices[j],
                          s.placements[j].transform.pos, s.boxes[j], gap);
}

size_t normalizeBins(std::vector<Placement> &placements) {
    std::vector<size_t> ids;
    for (const auto &p : placements)
        ids.push_back(p.bin);
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    for (auto &p : placements)
        p.bin = std::lower_bound(ids.begin(), ids.end(), p.bin) - ids.begin();
    return ids.size();
}

std::vector<double> coordinates(double size, double extent, int axis, const State &s, const std::vector<size_t> &placed,
                                size_t bin, double gap) {
    std::vector<double> out = {0, std::max(0.0, size - extent)};
    auto get = [axis](const Vec3 &v) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; };
    for (size_t j : placed)
        if (s.placements[j].bin == bin) {
            out.push_back(get(s.boxes[j].max) + gap);
            out.push_back(get(s.boxes[j].min) - extent - gap);
            // Same lower corner allows concave parts to interlock; geometry decides feasibility.
            out.push_back(get(s.boxes[j].min));
        }
    out.erase(
        std::remove_if(out.begin(), out.end(), [&](double v) { return v < 0 || v + extent > size + 1e-10 * size; }),
        out.end());
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

State makeState(const std::vector<Mesh> &meshes, const std::vector<Placement> &placements) {
    State s;
    s.placements = placements;
    s.matrices.resize(meshes.size());
    s.boxes.resize(meshes.size());
    for (size_t i = 0; i < meshes.size(); ++i) {
        s.matrices[i] = placements[i].transform.getMatrix();
        s.boxes[i] = computeTightAABB(meshes[i], s.matrices[i], placements[i].transform.pos);
    }
    return s;
}

struct Conflicts {
    std::vector<std::vector<int>> pairs;
    std::vector<int> degree;
    int total = 0;
};

Conflicts countConflicts(const std::vector<Mesh> &meshes, const State &state, double gap) {
    const size_t n = meshes.size();
    Conflicts out{std::vector<std::vector<int>>(n, std::vector<int>(n, 0)), std::vector<int>(n, 0), 0};
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < i; ++j) {
            const int value = conflict(meshes, state, i, j, gap);
            out.pairs[i][j] = out.pairs[j][i] = value;
            out.degree[i] += value;
            out.degree[j] += value;
            out.total += value;
        }
    return out;
}

void replaceConflictRow(Conflicts &conflicts, size_t i, const std::vector<int> &row) {
    int degree = 0;
    for (size_t j = 0; j < row.size(); ++j)
        if (j != i) {
            const int delta = row[j] - conflicts.pairs[i][j];
            conflicts.total += delta;
            conflicts.degree[j] += delta;
            degree += row[j];
            conflicts.pairs[i][j] = conflicts.pairs[j][i] = row[j];
        }
    conflicts.degree[i] = degree;
}

// Randomized constructive search with bounded backtracking. Every prefix is
// feasible. Frames retain untried placements so a dead end can undo up to three
// insertions before restarting with a new order. There is no temperature, energy
// objective, or acceptance of colliding placements.
bool constructAttempt(const std::vector<Mesh> &meshes, const PackingOptions &o,
                      const std::vector<std::vector<Oriented>> &orientations, State &state, size_t target,
                      std::mt19937 &rng, const std::function<bool()> &expired, PackingResult &stats,
                      const std::function<void()> &report) {
    struct Candidate {
        size_t bin;
        Oriented orientation;
        Vec3 minimum;
    };
    struct Frame {
        std::vector<Candidate> candidates;
        size_t next = 0;
    };
    const size_t n = meshes.size();
    std::uniform_real_distribution<double> unit(0, 1), angle(-180, 180);
    while (!expired()) {
        ++stats.restarts;
        std::vector<size_t> order(n);
        std::iota(order.begin(), order.end(), 0);
        std::vector<double> priority(n);
        for (size_t i = 0; i < n; ++i)
            priority[i] = orientations[i][0].box.volume() * (0.5 + unit(rng));
        if (stats.restarts % 3 == 0)
            std::shuffle(order.begin(), order.end(), rng);
        else
            std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return priority[a] > priority[b]; });
        std::vector<size_t> placed;
        std::vector<Frame> frames(n);
        size_t depth = 0, furthest = 0, backtracks = 0;
        while (!expired()) {
            if (depth == n)
                return true;
            const size_t i = order[depth];
            auto &frame = frames[depth];
            if (frame.candidates.empty()) {
                auto rotations = orientations[i];
                // Like annealing, allow free rotations as well as the shared 24
                // axis orientations. Tight bounds are computed on the actual mesh.
                for (int j = 0; j < 2 && !expired(); ++j) {
                    auto r = orient(meshes[i], {angle(rng), angle(rng), angle(rng)});
                    if (fits(r.box, o.size))
                        rotations.push_back(r);
                }
                std::shuffle(rotations.begin(), rotations.end(), rng);
                size_t usedBins = 0;
                for (size_t j : placed)
                    usedBins = std::max(usedBins, state.placements[j].bin + 1);
                // Empty containers are interchangeable: try only the first empty one.
                const size_t allowedBins = std::min(target, usedBins + 1);
                for (size_t bin = 0; bin < allowedBins && !expired(); ++bin) {
                    for (const auto &r : rotations) {
                        if (expired())
                            break;
                        const Vec3 e = r.box.max - r.box.min;
                        auto xs = coordinates(o.size.x, e.x, 0, state, placed, bin, o.gap);
                        auto ys = coordinates(o.size.y, e.y, 1, state, placed, bin, o.gap);
                        auto zs = coordinates(o.size.z, e.z, 2, state, placed, bin, o.gap);
                        if (xs.empty() || ys.empty() || zs.empty())
                            continue;
                        const size_t count = xs.size() * ys.size() * zs.size();
                        std::vector<Vec3> points;
                        if (count <= 96) {
                            for (double z : zs)
                                for (double y : ys)
                                    for (double x : xs)
                                        points.push_back({x, y, z});
                        } else {
                            points.push_back({0, 0, 0});
                            for (int j = 0; j < 95; ++j)
                                points.push_back({xs[rng() % xs.size()], ys[rng() % ys.size()], zs[rng() % zs.size()]});
                        }
                        // A few near-face positions allow nesting when tight
                        // bounding boxes overlap. Geometry still decides validity.
                        for (int j = 0; j < 8; ++j) {
                            const Vec3 p = points[rng() % points.size()];
                            points.push_back(
                                {std::clamp(p.x + (unit(rng) - 0.5) * e.x, 0.0, std::max(0.0, o.size.x - e.x)),
                                 std::clamp(p.y + (unit(rng) - 0.5) * e.y, 0.0, std::max(0.0, o.size.y - e.y)),
                                 std::clamp(p.z + (unit(rng) - 0.5) * e.z, 0.0, std::max(0.0, o.size.z - e.z))});
                        }
                        for (const auto &p : points)
                            frame.candidates.push_back({bin, r, p});
                    }
                }
                // Greedy compactness with seeded perturbation: different restarts
                // try different corners, orientations and container assignments.
                std::shuffle(frame.candidates.begin(), frame.candidates.end(), rng);
                if (stats.restarts % 3 != 0)
                    std::stable_sort(frame.candidates.begin(), frame.candidates.end(),
                                     [&](const Candidate &a, const Candidate &b) {
                                         auto score = [&](const Candidate &c) {
                                             return c.bin * 3.0 + c.minimum.x / o.size.x + c.minimum.y / o.size.y +
                                                    c.minimum.z / o.size.z;
                                         };
                                         return score(a) < score(b);
                                     });
            }
            bool found = false;
            while (frame.next < frame.candidates.size() && !expired()) {
                const auto &candidate = frame.candidates[frame.next++];
                place(state, i, candidate.bin, candidate.orientation, candidate.minimum);
                ++stats.iterations;
                found = true;
                for (size_t j : placed)
                    if (conflict(meshes, state, i, j, o.gap)) {
                        found = false;
                        break;
                    }
                if (found)
                    break;
            }
            if (found) {
                placed.push_back(i);
                ++depth;
                furthest = std::max(furthest, depth);
                if (depth < n)
                    frames[depth] = {};
            } else {
                frames[depth] = {};
                if (depth == 0 || furthest - depth >= 3 || ++backtracks > 32)
                    break;
                --depth;
                placed.pop_back();
            }
            report();
        }
    }
    return false;
}

// Tie-breaker within a fixed bin count. Normalization keeps the search independent
// of coordinate units (apart from floating-point tolerances).
double cornerCost(const State &s, size_t i, const Vec3 &size) {
    const Vec3 c = s.boxes[i].center();
    return 0.02 * (c.x / size.x + c.y / size.y + c.z / size.z) / s.placements.size();
}
} // namespace

PackingResult packMeshes(const std::vector<Mesh> &meshes, const PackingOptions &o, const ProgressCallback &progress) {
    if (meshes.empty())
        throw std::invalid_argument("At least one mesh is required");
    if (!std::isfinite(o.size.x) || !std::isfinite(o.size.y) || !std::isfinite(o.size.z) || o.size.x <= 0 ||
        o.size.y <= 0 || o.size.z <= 0 || !std::isfinite(o.gap) || o.gap < 0 || !std::isfinite(o.seconds) ||
        o.seconds <= 0)
        throw std::invalid_argument("Invalid container dimensions, gap or time limit");
    const auto preparationStart = Clock::now();
    const size_t n = meshes.size();
    std::mt19937 rng(o.seed);
    std::uniform_real_distribution<double> unit(0, 1), angle(-180, 180);
    auto randomRotation = [&]() { return Vec3{angle(rng), angle(rng), angle(rng)}; };
    std::vector<std::vector<Oriented>> orientations(n);
    // Find a guaranteed feasible starting state: one fitted item per bin.
    // Preprocessing is outside the improvement time budget, like loading/BVH building.
    State baseline;
    baseline.placements.resize(n);
    baseline.matrices.resize(n);
    baseline.boxes.resize(n);
    for (size_t i = 0; i < n; ++i) {
        for (int x = 0; x < 4; ++x)
            for (int y = 0; y < 4; ++y)
                for (int z = 0; z < 4; ++z) {
                    Oriented candidate = orient(meshes[i], {90.0 * x, 90.0 * y, 90.0 * z});
                    if (!fits(candidate.box, o.size))
                        continue;
                    bool duplicate = false;
                    for (const auto &old : orientations[i]) {
                        double delta = 0;
                        for (int r = 0; r < 3; ++r)
                            for (int c = 0; c < 3; ++c)
                                delta += std::abs(old.matrix.m[r][c] - candidate.matrix.m[r][c]);
                        if (delta < 1e-9) {
                            duplicate = true;
                            break;
                        }
                    }
                    if (!duplicate)
                        orientations[i].push_back(candidate);
                }
        if (orientations[i].empty()) {
            for (int attempt = 0; attempt < 1024; ++attempt) {
                auto candidate = orient(meshes[i], randomRotation());
                if (fits(candidate.box, o.size)) {
                    orientations[i].push_back(candidate);
                    break;
                }
            }
        }
        if (orientations[i].empty())
            throw std::runtime_error("No fitting orientation found for input " + std::to_string(i + 1) +
                                     "; increase the container dimensions or pre-orient the model");
        place(baseline, i, i, orientations[i][0], {0, 0, 0});
    }
    const auto start = Clock::now();
    auto elapsed = [&]() { return std::chrono::duration<double>(Clock::now() - start).count(); };
    auto expired = [&]() { return elapsed() >= o.seconds; };
    int lastPercent = -1;
    size_t lastTarget = 0;
    auto report = [&](size_t bins, size_t target, bool done = false) {
        const int percent = done ? 100 : static_cast<int>(std::min(99.0, 100 * elapsed() / o.seconds));
        if (progress && (percent != lastPercent || target != lastTarget)) {
            lastPercent = percent;
            lastTarget = target;
            progress(percent, bins, target);
        }
    };
    report(n, 0);
    // Spend at most 20% of the total budget constructing a feasible fallback.
    // This is an upper bound, not the order in which container counts are tested.
    auto greedyExpired = [&]() { return elapsed() >= 0.2 * o.seconds; };

    // First-fit decreasing, using face-aligned candidate positions. Actual mesh
    // distance checks (not bounding-box packing) decide whether a placement is valid.
    State greedy = baseline;
    std::vector<size_t> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(),
                     [&](size_t a, size_t b) { return baseline.boxes[a].volume() > baseline.boxes[b].volume(); });
    std::vector<size_t> placed;
    size_t bins = 0;
    bool baselineTimedOut = false;
    for (size_t i : order) {
        bool found = false;
        for (size_t bin = 0; bin < bins && !found && !greedyExpired(); ++bin) {
            for (const auto &rotation : orientations[i]) {
                if (found || greedyExpired())
                    break;
                const Vec3 e = rotation.box.max - rotation.box.min;
                const auto xs = coordinates(o.size.x, e.x, 0, greedy, placed, bin, o.gap);
                const auto ys = coordinates(o.size.y, e.y, 1, greedy, placed, bin, o.gap);
                const auto zs = coordinates(o.size.z, e.z, 2, greedy, placed, bin, o.gap);
                for (double z : zs) {
                    if (found || greedyExpired())
                        break;
                    for (double y : ys) {
                        if (found || greedyExpired())
                            break;
                        for (double x : xs) {
                            if (greedyExpired())
                                break;
                            place(greedy, i, bin, rotation, {x, y, z});
                            bool valid = true;
                            for (size_t j : placed)
                                if (conflict(meshes, greedy, i, j, o.gap)) {
                                    valid = false;
                                    break;
                                }
                            if (valid) {
                                found = true;
                                break;
                            }
                        }
                    }
                }
            }
        }
        if (!found) {
            baselineTimedOut = baselineTimedOut || greedyExpired();
            place(greedy, i, bins++, orientations[i][0], {0, 0, 0});
        }
        placed.push_back(i);
        report(bins + n - placed.size(), 0);
    }
    PackingResult best;
    best.placements = greedy.placements;
    best.binCount = bins;
    best.baselineBinCount = bins;
    best.baselineTimedOut = baselineTimedOut;
    best.baselineSeconds = elapsed();
    best.preparationSeconds = std::chrono::duration<double>(start - preparationStart).count();
    const size_t upperBound = bins;
    State carry = greedy;
    for (auto &placement : carry.placements)
        placement.bin = 0;
    Conflicts carryConflicts;

    // Try one container, then two, and so on. An unsuccessful time-limited
    // attempt is not a proof of infeasibility. The greedy layout remains available
    // if the total time budget expires before a successful attempt.
    for (size_t target = 1; target <= upperBound; ++target) {
        best.attemptedBinCounts.push_back(target);
        report(best.binCount, target);
        if (target == upperBound) {
            // We already have a validated construction at this count.
            best.searchSeconds = elapsed();
            validatePacking(meshes, o, best);
            report(best.binCount, target, true);
            return best;
        }
        // The known feasible upper bound needs no search allocation. Share the
        // remaining budget among the smaller counts that have not yet been tried.
        const double attemptDeadline =
            elapsed() + std::max(0.0, o.seconds - elapsed()) / static_cast<double>(upperBound - target);
        auto attemptExpired = [&]() { return elapsed() >= attemptDeadline; };
        if (o.algorithm == PackingAlgorithm::Constructive) {
            State state = baseline;
            if (constructAttempt(meshes, o, orientations, state, target, rng, attemptExpired, best,
                                 [&]() { report(best.binCount, target); })) {
                best.placements = state.placements;
                best.binCount = normalizeBins(best.placements);
                best.searchSeconds = elapsed();
                validatePacking(meshes, o, best);
                report(best.binCount, target, true);
                return best;
            }
            if (expired() && target + 1 < upperBound) {
                best.usedFallback = true;
                break;
            }
            continue;
        }
        State state = carry;
        Conflicts conflicts;
        if (target == 1) {
            conflicts = countConflicts(meshes, state, o.gap);
        } else {
            conflicts = carryConflicts;
            // Open the new container by moving the model involved in the most
            // violations. Keep the best partial layout from the preceding attempt.
            const size_t i =
                std::max_element(conflicts.degree.begin(), conflicts.degree.end()) - conflicts.degree.begin();
            state.placements[i].bin = target - 1;
            replaceConflictRow(conflicts, i, std::vector<int>(n, 0));
        }
        State partial = state;
        Conflicts partialConflicts = conflicts;
        const size_t phaseLength = std::max<size_t>(1000, 500 * n);
        size_t step = 0;
        while (true) {
            if (conflicts.total == 0) {
                best.placements = state.placements;
                best.binCount = normalizeBins(best.placements);
                best.searchSeconds = elapsed();
                validatePacking(meshes, o, best);
                report(best.binCount, target, true);
                return best;
            }
            if (attemptExpired())
                break;
            if (step == phaseLength) {
                ++best.restarts;
                // Restart from the best partial solution, reheat, and continue
                // exploring this same count until its allotted time expires.
                state = partial;
                conflicts = partialConflicts;
                step = 0;
            }
            size_t i = rng() % n;
            if (unit(rng) < 0.8) {
                // Most proposals repair a model that is actually in conflict.
                // Occasional unrestricted choices can move a blocking neighbour.
                std::discrete_distribution<size_t> choose(conflicts.degree.begin(), conflicts.degree.end());
                i = choose(rng);
            }
            const Placement oldPlacement = state.placements[i];
            const Mat3 oldMatrix = state.matrices[i];
            const AABB oldBox = state.boxes[i];
            const double oldCost = cornerCost(state, i, o.size);
            const double p = static_cast<double>(step++) / phaseLength;
            Oriented rotation{oldPlacement.transform.rot, oldMatrix, {}};
            rotation.box = computeTightAABB(meshes[i], oldMatrix, {0, 0, 0});
            const double roll = unit(rng);
            if (roll < 0.15)
                rotation = orient(meshes[i], randomRotation());
            else if (roll < 0.35)
                rotation = orientations[i][rng() % orientations[i].size()];
            if (!fits(rotation.box, o.size))
                continue;
            const Vec3 e = rotation.box.max - rotation.box.min;
            size_t bin = oldPlacement.bin;
            Vec3 minimum = oldBox.min;
            if (roll > 0.75 && target > 1) {
                // Any other container is eligible; lighter containers are more
                // likely destinations, but the real geometry decides acceptance.
                std::vector<double> weights(target, 1.0);
                for (size_t j = 0; j < n; ++j)
                    if (j != i)
                        weights[state.placements[j].bin] += 1.0;
                for (double &weight : weights)
                    weight = 1.0 / weight;
                weights[oldPlacement.bin] = 0;
                std::discrete_distribution<size_t> destination(weights.begin(), weights.end());
                bin = destination(rng);
                minimum = {unit(rng) * std::max(0.0, o.size.x - e.x), unit(rng) * std::max(0.0, o.size.y - e.y),
                           unit(rng) * std::max(0.0, o.size.z - e.z)};
            } else {
                std::normal_distribution<double> normal(0, 0.2 * std::max(0.02, 1 - p));
                minimum = minimum + Vec3{normal(rng) * o.size.x, normal(rng) * o.size.y, normal(rng) * o.size.z};
            }
            // Snap one coordinate to a neighbouring face or a container wall.
            if (unit(rng) < 0.45) {
                const int axis = rng() % 3;
                const size_t j = rng() % n;
                auto get = [axis](const Vec3 &v) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; };
                double value = unit(rng) < 0.5 ? 0 : get(o.size) - get(e);
                if (j != i && state.placements[j].bin == bin)
                    value =
                        unit(rng) < 0.5 ? get(state.boxes[j].max) + o.gap : get(state.boxes[j].min) - get(e) - o.gap;
                if (axis == 0)
                    minimum.x = value;
                else if (axis == 1)
                    minimum.y = value;
                else
                    minimum.z = value;
            }
            minimum.x = std::clamp(minimum.x, 0.0, std::max(0.0, o.size.x - e.x));
            minimum.y = std::clamp(minimum.y, 0.0, std::max(0.0, o.size.y - e.y));
            minimum.z = std::clamp(minimum.z, 0.0, std::max(0.0, o.size.z - e.z));
            place(state, i, bin, rotation, minimum);
            std::vector<int> row(n, 0);
            int delta = 0;
            for (size_t j = 0; j < n; ++j)
                if (j != i) {
                    row[j] = conflict(meshes, state, i, j, o.gap);
                    delta += row[j] - conflicts.pairs[i][j];
                }
            const double change = delta + cornerCost(state, i, o.size) - oldCost;
            const double temperature = 0.5 * std::pow(0.002, p);
            if (change <= 0 || unit(rng) < std::exp(-change / temperature)) {
                replaceConflictRow(conflicts, i, row);
                if (conflicts.total < partialConflicts.total) {
                    partial = state;
                    partialConflicts = conflicts;
                }
            } else {
                state.placements[i] = oldPlacement;
                state.matrices[i] = oldMatrix;
                state.boxes[i] = oldBox;
            }
            ++best.iterations;
            report(best.binCount, target);
        }
        carry = std::move(partial);
        carryConflicts = std::move(partialConflicts);
        if (expired() && target + 1 < upperBound) {
            best.usedFallback = true;
            break;
        }
    }
    best.searchSeconds = elapsed();
    validatePacking(meshes, o, best);
    report(best.binCount, best.attemptedBinCounts.back(), true);
    return best;
}

void validatePacking(const std::vector<Mesh> &meshes, const PackingOptions &o, const PackingResult &result) {
    if (result.placements.size() != meshes.size())
        throw std::runtime_error("Invalid placement count");
    State state = makeState(meshes, result.placements);
    const double eps = 1e-9 * std::max({o.size.x, o.size.y, o.size.z});
    for (size_t i = 0; i < meshes.size(); ++i) {
        const auto &b = state.boxes[i];
        if (result.placements[i].bin >= result.binCount || b.min.x < -eps || b.min.y < -eps || b.min.z < -eps ||
            b.max.x > o.size.x + eps || b.max.y > o.size.y + eps || b.max.z > o.size.z + eps)
            throw std::runtime_error("Internal error: placement outside container");
        for (size_t j = 0; j < i; ++j)
            if (conflict(meshes, state, i, j, o.gap))
                throw std::runtime_error("Internal error: output violates mesh clearance");
    }
}

double binDisplayStride(const PackingOptions &options) {
    return options.size.x + std::max(options.gap, options.size.x * 0.1);
}
