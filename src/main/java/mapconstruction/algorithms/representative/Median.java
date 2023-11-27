package mapconstruction.algorithms.representative;

import mapconstruction.trajectories.FullTrajectory;
import mapconstruction.trajectories.Subtrajectory;
import mapconstruction.util.GeometryUtil;
import mapconstruction.util.Pair;

import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.util.List;
import java.util.*;

enum CCW {CCW, CoLinear, CW}

public final class Median {

    static Subtrajectory t1 = new Subtrajectory(new FullTrajectory(Arrays.asList(
            new Point2D.Double(0, 0)
            , new Point2D.Double(1, 1)
            , new Point2D.Double(2, 1)
    )));

    static Subtrajectory t2 = new Subtrajectory(new FullTrajectory(Arrays.asList(
            new Point2D.Double(1, 2)
            , new Point2D.Double(2, 0)
    )));
    static Subtrajectory t3 = new Subtrajectory(new FullTrajectory(Arrays.asList(
            new Point2D.Double(1.5, 0)
            , new Point2D.Double(2, 2)
    )));
    static Set<Subtrajectory> testTs = new HashSet(Arrays.asList(t1, t2, t3));
    static Subtrajectory testMedian = representativeTrajectory(testTs, new Point2D.Double(0, 0), new Point2D.Double(3, 3));

    public static void main(String args[]) {
        System.out.print(testMedian);
    }

    public static Subtrajectory representativeTrajectory(Set<Subtrajectory> trajectories, Point2D start, Point2D end) {
        List<Subtrajectory> ts = new ArrayList(trajectories.size());
        for (Subtrajectory t : trajectories) ts.add(appendEndPoints(t, start, end));


        List<Point2D> repr = new LinkedList();
        repr.add(start);

        Subtrajectory t = findStartingEdge(ts, start);
        P<Subtrajectory> s = new P(start, t);

        //inv: the starting vertex s does not lie on any of the trajectories
        //
        do {
            P<Pair<Subtrajectory, Integer>> v = findNextVertex(s, ts);
            repr.add(v);
            //System.out.println(v.toString());

            // drop part of the subtrajectories so that s is no longer on them
            s.idx.dropInPlace(1);
            if (v.idx.getFirst() != s.idx)
                v.idx.getFirst().dropInPlace(v.idx.getSecond() + 1);

            s = new P(v, v.idx.getFirst());
        } while (!s.equals(end));

        return new Subtrajectory(new FullTrajectory(repr));
    }

    static P<Pair<Subtrajectory, Integer>> findNextVertex(P<Subtrajectory> start, List<Subtrajectory> ts) {
        Subtrajectory t = start.idx;
        Line2D seg = new Line2D.Double(start, t.getFirstPoint());
        P<Pair<Subtrajectory, Integer>> closest = new P(seg.getP2(), new Pair(t, 1));

        for (Subtrajectory tp : ts) {
            if (!(t == tp)) { // this should really just be t != tp
                P<Pair<Subtrajectory, Integer>> p = findNextIntersection(seg, tp);
                if (p != null)
                    if (start.distanceSq(p) < start.distanceSq(closest))
                        closest = p;
            }
        }
        return closest;
    }

    static P<Pair<Subtrajectory, Integer>> findNextIntersection(Line2D s, Subtrajectory t) {
        for (int i = 0; i < t.numEdges(); i++)
            if (s.intersectsLine(t.getEdge(i))) {
                // we already know the segments supposedly intersect, so compute the intersection point
                Point2D.Double p = GeometryUtil.intersectionPoint(s, t.getEdge(i));
                if (p != null) {
                    return new P(p, new Pair(t, i));
                }
//				System.out.println("error..");
            }
        return null;
    }

    public static Subtrajectory findStartingEdge(List<Subtrajectory> trajectories, Point2D start) {
        int n = trajectories.size();
        ArrayList<P<Subtrajectory>> pts = new ArrayList(n);
        for (Subtrajectory t : trajectories) pts.add(new P(t.getFirstPoint(), t));

        pts.sort(new Arround(start));
        int i = n % 2 == 0 ? (n / 2) - 1 : n / 2;
        return pts.get(i).idx;
    }

    // trim the start and add end point if the trajectory does not have them already
    private static Subtrajectory appendEndPoints(Subtrajectory st, Point2D start, Point2D end) {
        Point2D p = st.getFirstPoint();
        Point2D q = st.getLastPoint();
        final int n = st.numPoints();
        List<Point2D> pts = new ArrayList(n + 2);
        List<Point2D> temp = new ArrayList(st.points());

        if (start.distanceSq(q) < start.distanceSq(p) && end.distanceSq(p) < end.distanceSq(q))
            Collections.reverse(temp);

        if (p.equals(start)) {
            pts.addAll(temp.subList(1, n));
        } else pts.addAll(temp);
        if (!(q.equals(end))) pts.add(end);

        return new Subtrajectory(new FullTrajectory(pts));
    }

    static class P<T> extends Point2D.Double {
        /**
         * point together with some value t.
         */
        private static final long serialVersionUID = -8464739006403341132L;
        T idx;

        public P(Point2D p, T i) {
            x = p.getX();
            y = p.getY();
            idx = i;
        }
    }

    static class Arround implements Comparator<Point2D> {
        Point2D me;

        public Arround(Point2D s) {
            this.me = s;
        }


        // counter clockwise quadrants
        int quadrant(Point2D p) {
            if (p.getX() > 0 && p.getY() >= 0) return 1;
            if (p.getX() <= 0 && p.getY() > 0) return 2;
            if (p.getX() < 0 && p.getY() <= 0) return 3;
            return 4;
        }

        // Given three points p q and r determine the orientation when going from p to r via q.
        CCW ccw(Point2D p, Point2D q, Point2D r) {
            Point2D u = new Point.Double(q.getX() - p.getX(), q.getY() - p.getY());
            Point2D v = new Point.Double(r.getX() - p.getX(), r.getY() - p.getY());

            double z = u.getX() * v.getY() - u.getY() * v.getX();

            if (z == 0) return CCW.CoLinear;
            if (z > 0) return CCW.CCW;
            else return CCW.CW;
        }

        @Override
        public int compare(Point2D pp, Point2D qq) {
            Point2D p = new Point.Double(pp.getX() - me.getX(), pp.getY() - me.getY());
            Point2D q = new Point.Double(qq.getX() - me.getX(), qq.getY() - me.getY());

            if (quadrant(p) < quadrant(q)) return -1;
            if (quadrant(p) > quadrant(q)) return 1;

            // same quadrant
            switch (ccw(me, pp, qq)) {
                case CoLinear:
                    return 0;
                case CCW:
                    return 1;
                default:
                    return -1;
            }
        }

    }


};
