/***
 * @Author: Xu.WANG raymondmgwx@gmail.com
 * @Date: 2023-01-11 14:46:17
 * @LastEditors: Xu.WANG raymondmgwx@gmail.com
 * @LastEditTime: 2023-01-14 11:11:02
 * @FilePath: \Kiri2D\core\include\kiri2d\physics\rigidbody\collision_handler.h
 * @Description:
 * @Copyright (c) 2023 by Xu.WANG raymondmgwx@gmail.com, All Rights Reserved.
 */
#ifndef _COLLISION_HANDLER_H_
#define _COLLISION_HANDLER_H_

#pragma once
#include <eventpp/eventdispatcher.h>
#include <kiri2d/physics/rigidbody/rigidbody.h>
#include <memory>

namespace KIRI2D::PHY::RIGIDBODY
{

  template <class RealType>
  class CollisionHandler;

  template <class RealType>
  static void
  Circle2Circle(const std::shared_ptr<CollisionHandler<RealType>> &handler,
                std::shared_ptr<RigidBody<RealType>> bodyA,
                std::shared_ptr<RigidBody<RealType>> bodyB)
  {

    // KIRI_LOG_DEBUG("Dispatch Circle2Circle!!");

    auto shape_a = std::dynamic_pointer_cast<Circle<RealType>>(bodyA->GetShape());
    auto shape_b = std::dynamic_pointer_cast<Circle<RealType>>(bodyB->GetShape());
    auto pos_a = bodyA->GetPosition();
    auto pos_b = bodyB->GetPosition();
    auto rad_a = shape_a->GetRadius();
    auto rad_b = shape_b->GetRadius();

    auto dir = pos_b - pos_a;
    auto dist = dir.length();
    auto radius = rad_a + rad_b;

    if (dist >= radius)
    {
      handler->SetContactNum(0);
      return;
    }

    handler->SetContactNum(1);

    if (dist == static_cast<RealType>(0.0))
    {
      handler->SetPenetration(rad_a);
      handler->SetContactDir(VectorX<2, RealType>(1.0, 0.0));
      handler->SetContactPoint(pos_a, 0);
    }
    else
    {
      // KIRI_LOG_DEBUG("contact!!");
      handler->SetPenetration(radius - dist);
      auto unit_dir = dir / dist;
      handler->SetContactDir(unit_dir);
      handler->SetContactPoint(unit_dir * rad_a + pos_a, 0);
    }
  }

  template <class RealType>
  static void
  Circle2Polygon(const std::shared_ptr<CollisionHandler<RealType>> &handler,
                 std::shared_ptr<RigidBody<RealType>> bodyA,
                 std::shared_ptr<RigidBody<RealType>> bodyB)
  {

    // KIRI_LOG_DEBUG("Dispatch Circle2Polygon!!");
    handler->SetContactNum(0);

    auto shape_a = std::dynamic_pointer_cast<Circle<RealType>>(bodyA->GetShape());
    auto shape_b = std::dynamic_pointer_cast<Polygon<RealType>>(bodyB->GetShape());

    auto pos_a = bodyA->GetPosition();
    auto pos_b = bodyB->GetPosition();
    auto rad_a = shape_a->GetRadius();

    // transform circle to polygon model space
    auto rotate_mat = shape_b->GetRotateMatrix();
    auto circle_center = rotate_mat.transposed() * (pos_a - pos_b);
    auto separation = -Huge<RealType>();
    auto face_normal = 0;
    auto vertices = shape_b->GetVertices();
    auto normals = shape_b->GetNormals();
    auto vertices_num = shape_b->GetVerticesNum();
    for (auto i = 0; i < vertices_num; ++i)
    {
      auto s = normals[i].dot(circle_center - vertices[i]);

      if (s > rad_a)
        return;

      if (s > separation)
      {
        separation = s;
        face_normal = i;
      }
    }

    auto v1 = vertices[face_normal];
    auto v2 = vertices[(face_normal + 1) % vertices_num];

    if (separation < MEpsilon<RealType>())
    {
      handler->SetContactNum(1);
      auto dir = -(rotate_mat * normals[face_normal]);
      handler->SetContactDir(dir);
      handler->SetContactPoint(dir * rad_a + pos_a, 0);
      handler->SetPenetration(rad_a);
      return;
    }

    auto dot1 = (circle_center - v1).dot(v2 - v1);
    auto dot2 = (circle_center - v2).dot(v1 - v2);
    handler->SetPenetration(rad_a - separation);

    if (dot1 <= static_cast<RealType>(0.0))
    {
      auto dir = circle_center - v1;
      if (dir.dot(dir) > rad_a * rad_a)
        return;

      handler->SetContactNum(1);
      auto n = v1 - circle_center;
      n = rotate_mat * n;
      if (n.length() > MEpsilon<RealType>())
        n.normalize();
      handler->SetContactDir(n);
      v1 = rotate_mat * v1 + pos_b;
      handler->SetContactPoint(v1, 0);
    }
    else if (dot2 <= static_cast<RealType>(0.0))
    {
      auto dir = circle_center - v2;
      if (dir.dot(dir) > rad_a * rad_a)
        return;

      handler->SetContactNum(1);
      auto n = v2 - circle_center;
      v2 = rotate_mat * v2 + pos_b;
      handler->SetContactPoint(v2, 0);
      n = rotate_mat * n;
      if (n.length() > MEpsilon<RealType>())
        n.normalize();
      handler->SetContactDir(n);
    }
    else
    {
      auto n = normals[face_normal];
      if ((circle_center - v1).dot(n) > rad_a)
        return;

      n = rotate_mat * n;
      handler->SetContactDir(-n);
      handler->SetContactPoint(handler->GetContactDir() * rad_a + pos_a, 0);
      handler->SetContactNum(1);
    }
  }

  template <class RealType>
  static void
  Polygon2Circle(const std::shared_ptr<CollisionHandler<RealType>> &handler,
                 std::shared_ptr<RigidBody<RealType>> bodyA,
                 std::shared_ptr<RigidBody<RealType>> bodyB)
  {
    Circle2Polygon<RealType>(handler, bodyB, bodyA);
    handler->SetContactDir(-handler->GetContactDir());
  }

  template <class RealType>
  static std::tuple<RealType, int> FindAxisLeastPenetration(
      const PolygonPtr<RealType> &shapeA,
      const PolygonPtr<RealType> &shapeB)
  {
    auto max_distance = -Huge<RealType>();
    auto max_index = -1;

    auto a_vertices_num = shapeA->GetVerticesNum();
    auto a_vertices = shapeA->GetVertices();
    auto a_normals = shapeA->GetNormals();

    auto a_rotmat = shapeA->GetRotateMatrix();
    auto b_rotmat = shapeB->GetRotateMatrix();

    auto pos_a = shapeA->GetBody().lock()->GetPosition();
    auto pos_b = shapeB->GetBody().lock()->GetPosition();

    for (auto i = 0; i < a_vertices_num; ++i)
    {
      auto face_a = a_rotmat * a_normals[i];
      auto face_a2b = b_rotmat.transposed() * face_a;

      auto extreme_along_b = shapeB->GetExtremePointAlongDir(-face_a2b);

      auto vert_a2b = b_rotmat.transposed() * (a_rotmat * a_vertices[i] + pos_a - pos_b);
      auto penetration = face_a2b.dot(extreme_along_b - vert_a2b);

      if (penetration > max_distance)
      {
        max_distance = penetration;
        max_index = i;
      }
    }

    return std::make_pair(max_distance, max_index);
  }

  template <class RealType>
  static std::vector<VectorX<2, RealType>> FindIncidentFace(
      const PolygonPtr<RealType> &referencePoly,
      const PolygonPtr<RealType> &incidentPoly,
      int reference_index)
  {
    auto incident_rotmat = incidentPoly->GetRotateMatrix();
    auto incident_vertices = incidentPoly->GetVertices();
    auto incident_pos = incidentPoly->GetBody().lock()->GetPosition();
    auto incident_vertnum = incidentPoly->GetVerticesNum();

    auto normal_ref2ind = incident_rotmat.transposed() * referencePoly->GetRotateMatrix() * referencePoly->GetNormals()[reference_index];

    auto incident_face = 0;
    auto min_dot = Huge<RealType>();
    for (auto i = 0; i < incident_vertnum; ++i)
    {
      auto dot = normal_ref2ind.dot(incidentPoly->GetNormals()[i]);
      if (dot < min_dot)
      {
        min_dot = dot;
        incident_face = i;
      }
    }

    std::vector<VectorX<2, RealType>> incident_faces{
        incident_rotmat * incident_vertices[incident_face] + incident_pos,
        incident_rotmat * incident_vertices[(incident_face + 1) % incident_vertnum] + incident_pos};
    return incident_faces;
  }

  template <class RealType>
  static std::tuple<int, std::vector<VectorX<2, RealType>>> Clip(
      VectorX<2, RealType> n, RealType c, std::vector<VectorX<2, RealType>> faces)
  {
    auto sp = 0;
    auto out = faces;

    auto d1 = n.dot(faces[0]) - c;
    auto d2 = n.dot(faces[1]) - c;

    if (d1 <= static_cast<RealType>(0.0))
      out[sp++] = faces[0];
    if (d2 <= static_cast<RealType>(0.0))
      out[sp++] = faces[1];

    if (d1 * d2 < static_cast<RealType>(0.0))
    {
      auto alpha = d1 / (d1 - d2);
      out[sp] = faces[0] + alpha * (faces[1] - faces[0]);
      ++sp;
    }

    KIRI_ASSERT(sp != 3);

    return std::make_pair(sp, out);
  }

  template <class RealType>
  static void
  Polygon2Polygon(const std::shared_ptr<CollisionHandler<RealType>> &handler,
                  std::shared_ptr<RigidBody<RealType>> bodyA,
                  std::shared_ptr<RigidBody<RealType>> bodyB)
  {
    // KIRI_LOG_DEBUG("Dispatch Polygon2Polygon!!");

    handler->SetContactNum(0);

    auto shape_a = std::dynamic_pointer_cast<Polygon<RealType>>(bodyA->GetShape());
    auto shape_b = std::dynamic_pointer_cast<Polygon<RealType>>(bodyB->GetShape());

    auto [penetration_a, face_a] = FindAxisLeastPenetration(shape_a, shape_b);
    if (penetration_a >= static_cast<RealType>(0.0))
      return;

    auto [penetration_b, face_b] = FindAxisLeastPenetration(shape_b, shape_a);
    if (penetration_b >= static_cast<RealType>(0.0))
      return;

    auto reference_index = face_a;
    auto flip = false;

    auto reference_poly = shape_a;
    auto incident_poly = shape_b;

    if (penetration_a < (penetration_b * static_cast<RealType>(0.95) + penetration_a * static_cast<RealType>(0.01)))
    {
      reference_index = face_b;
      flip = true;
      reference_poly = shape_b;
      incident_poly = shape_a;
    }

    auto incident_face = FindIncidentFace(reference_poly, incident_poly, reference_index);

    auto reference_vertices = reference_poly->GetVertices();
    auto reference_vertnum = reference_poly->GetVerticesNum();
    auto reference_rotmat = reference_poly->GetRotateMatrix();
    auto reference_pos = reference_poly->GetBody().lock()->GetPosition();
    auto v1 = reference_rotmat * reference_vertices[reference_index] + reference_pos;
    auto v2 = reference_rotmat * reference_vertices[(reference_index + 1) % reference_vertnum] + reference_pos;

    auto side_plane_normal = v2 - v1;

    if (side_plane_normal.length() > MEpsilon<RealType>())
      side_plane_normal.normalize();

    auto reference_face_normal = VectorX<2, RealType>(side_plane_normal.y, -side_plane_normal.x);

    auto refC = reference_face_normal.dot(v1);
    auto negSide = -side_plane_normal.dot(v1);
    auto posSide = side_plane_normal.dot(v2);

    auto [sp1, clip_incident_face1] = Clip(-side_plane_normal, negSide, incident_face);
    if (sp1 < 2)
      return;

    auto [sp2, clip_incident_face2] = Clip(side_plane_normal, posSide, clip_incident_face1);
    if (sp2 < 2)
      return;

    handler->SetContactDir(flip ? -reference_face_normal : reference_face_normal);

    auto cp = 0;
    auto separation = reference_face_normal.dot(clip_incident_face2[0]) - refC;
    if (separation <= static_cast<RealType>(0.0))
    {
      handler->SetContactPoint(clip_incident_face2[0], cp);
      handler->SetPenetration(-separation);
      ++cp;
    }
    else
      handler->SetPenetration(0);

    separation = reference_face_normal.dot(clip_incident_face2[1]) - refC;
    if (separation <= static_cast<RealType>(0.0))
    {
      handler->SetContactPoint(clip_incident_face2[1], cp);
      handler->SetPenetration(handler->GetPenetration() - separation);
      ++cp;

      handler->SetPenetration(handler->GetPenetration() / static_cast<RealType>(cp));
    }

    handler->SetContactNum(cp);
  }

  template <class RealType>
  class CollisionDispatchInternal
  {
  public:
    CollisionDispatchInternal()
    {
      mDispatcher.appendListener(
          std::make_pair(ShapeType::CIRCLE, ShapeType::CIRCLE),
          &Circle2Circle<RealType>);

      mDispatcher.appendListener(
          std::make_pair(ShapeType::CIRCLE, ShapeType::POLYGON),
          &Circle2Polygon<RealType>);

      mDispatcher.appendListener(
          std::make_pair(ShapeType::POLYGON, ShapeType::CIRCLE),
          &Polygon2Circle<RealType>);

      mDispatcher.appendListener(
          std::make_pair(ShapeType::POLYGON, ShapeType::POLYGON),
          &Polygon2Polygon<RealType>);
    }

    void Dispatch(const std::shared_ptr<CollisionHandler<RealType>> &handler,
                  std::shared_ptr<RigidBody<RealType>> bodyA,
                  std::shared_ptr<RigidBody<RealType>> bodyB)
    {
      mDispatcher.dispatch(std::make_pair(bodyA->GetShape()->GetType(),
                                          bodyB->GetShape()->GetType()),
                           handler, bodyA, bodyB);
    }

  private:
    eventpp::EventDispatcher<
        std::pair<int, int>,
        void(const std::shared_ptr<CollisionHandler<RealType>> &,
             std::shared_ptr<RigidBody<RealType>>,
             std::shared_ptr<RigidBody<RealType>>)>
        mDispatcher;
  };

  template <class RealType>
  class CollisionDispatcher
  {
  public:
    CollisionDispatcher(CollisionDispatcher &other) = delete;

    void operator=(const CollisionDispatcher &) = delete;

    static std::shared_ptr<CollisionDispatchInternal<RealType>> GetInstance()
    {
      if (mInstance == nullptr)
      {
        mInstance = std::make_shared<CollisionDispatchInternal<RealType>>();
      }
      return mInstance;
    }

  private:
    inline static std::shared_ptr<CollisionDispatchInternal<RealType>> mInstance =
        nullptr;
  };

  template <class RealType>
  class CollisionHandler
      : public std::enable_shared_from_this<CollisionHandler<RealType>>
  {
  public:
    explicit CollisionHandler(const std::shared_ptr<RigidBody<RealType>> &bodyA,
                              const std::shared_ptr<RigidBody<RealType>> &bodyB)
        : mBodyA(bodyA), mBodyB(bodyB)
    {
      // this->CheckContactInfo();
    }

    virtual ~CollisionHandler() {}

    const int GetContactNum() const { return mContactNum; }
    void SetContactNum(int num) { mContactNum = num; }

    void SetContactDir(VectorX<2, RealType> dir) { mContactDir = dir; }
    void SetPenetration(RealType val) { mPenetration = val; }
    void SetContactPoint(VectorX<2, RealType> p, int idx) { mContactPoints[idx] = p; }

    const RealType GetPenetration() const { return mPenetration; }
    const VectorX<2, RealType> &GetContactDir() const { return mContactDir; }
    const RealType GetMixRestitution() const { return mMixRestitution; }
    const RealType GetMixStaticFriction() const { return mMixStaticFriction; }
    const RealType GetMixDynamicFriction() const { return mMixDynamicFriction; }

    void ComputeImpluse()
    {
      auto a_im = mBodyA->GetInvMass();
      auto b_im = mBodyB->GetInvMass();
      if (std::abs(a_im + b_im) < MEpsilon<RealType>())
      {
        InfiniteMassCorrection();
        return;
      }

      for (auto i = 0; i < mContactNum; i++)
      {

        auto ra = mContactPoints[i] - mBodyA->GetPosition();
        auto rb = mContactPoints[i] - mBodyB->GetPosition();

        auto rv = mBodyB->GetVelocity() + VectorX<2, RealType>(-mBodyB->GetAngularVelocity() * rb.y, mBodyB->GetAngularVelocity() * rb.x) - mBodyA->GetVelocity() - VectorX<2, RealType>(-mBodyA->GetAngularVelocity() * ra.y, mBodyA->GetAngularVelocity() * ra.x);

        auto contact_vel = rv.dot(mContactDir);
        if (contact_vel > 0)
          return;

        auto ra_cross_n = ra.cross(mContactDir);
        auto rb_cross_n = rb.cross(mContactDir);
        auto inverse_mass_sum = a_im + b_im + ra_cross_n * ra_cross_n * mBodyA->GetInvInertia() + rb_cross_n * rb_cross_n * mBodyB->GetInvInertia();

        auto j = -(static_cast<RealType>(1.0) + mMixRestitution) * contact_vel;
        j /= inverse_mass_sum;
        j /= static_cast<RealType>(mContactNum);

        auto impulse = mContactDir * j;
        // KIRI_LOG_DEBUG("Compute first impulse; impulse={0},{1}; j={2};inverse_mass_sum={3};contact_vel={4}",
        //                impulse.x, impulse.y, j, inverse_mass_sum, contact_vel);

        mBodyA->ApplyImpulse(-impulse, ra);
        mBodyB->ApplyImpulse(impulse, rb);

        // friction
        rv = mBodyB->GetVelocity() + VectorX<2, RealType>(-mBodyB->GetAngularVelocity() * rb.y, mBodyB->GetAngularVelocity() * rb.x) - mBodyA->GetVelocity() - VectorX<2, RealType>(-mBodyA->GetAngularVelocity() * ra.y, mBodyA->GetAngularVelocity() * ra.x);

        auto t = rv - (mContactDir * rv.dot(mContactDir));
        if (t.length() > MEpsilon<RealType>())
          t.normalize();
        // KIRI_LOG_DEBUG("Computejt pre; rv={0},{1}; t={2},{3}", rv.x, rv.y, t.x, t.y);
        // KIRI_LOG_DEBUG("mContactDir={0},{1}; rv.dot(mContactDir)={2}", mContactDir.x, mContactDir.y, rv.dot(mContactDir));

        auto jt = -rv.dot(t);
        jt /= inverse_mass_sum;
        jt /= static_cast<RealType>(mContactNum);
        // KIRI_LOG_DEBUG("Computejt; jt={0}; inverse_mass_sum={1}", jt, inverse_mass_sum);

        if (std::abs(jt) < MEpsilon<RealType>())
          return;

        auto tangent_impulse = t * -j * mMixDynamicFriction;
        if (std::abs(jt) < j * mMixStaticFriction)
          tangent_impulse = t * jt;

        // KIRI_LOG_DEBUG("ComputeImpluse; tangent_impulse={0},{1}, ra={2},{3}, rb={4},{5} ", tangent_impulse.x, tangent_impulse.y, ra.x, ra.y, rb.x, rb.y);

        mBodyA->ApplyImpulse(-tangent_impulse, ra);
        mBodyB->ApplyImpulse(tangent_impulse, rb);
      }
    }

    void PositionalCorrection()
    {
      auto k_slop = static_cast<RealType>(0.05);
      auto percent = static_cast<RealType>(0.4);
      auto correction = (std::max(mPenetration - k_slop, static_cast<RealType>(0.0)) / (mBodyA->GetInvMass() + mBodyB->GetInvMass())) * mContactDir * percent;
      mBodyA->AddPosition(-correction * mBodyA->GetInvMass());
      mBodyB->AddPosition(correction * mBodyB->GetInvMass());
    }

    void ComputeContactSurfaceInfo(RealType dt, const VectorX<2, RealType> &gravity)
    {
      if (!mContactNum)
        return;

      mMixRestitution = std::min(mBodyA->GetRestitution(), mBodyB->GetRestitution());
      mMixStaticFriction = std::sqrt(mBodyA->GetStaticFriction() * mBodyA->GetStaticFriction() + mBodyB->GetStaticFriction() * mBodyB->GetStaticFriction());
      mMixDynamicFriction = std::sqrt(mBodyA->GetDynamicFriction() * mBodyA->GetDynamicFriction() + mBodyB->GetDynamicFriction() * mBodyB->GetDynamicFriction());

      for (auto i = 0; i < mContactNum; i++)
      {

        auto ra = mContactPoints[i] - mBodyA->GetPosition();
        auto rb = mContactPoints[i] - mBodyB->GetPosition();

        auto rv = mBodyB->GetVelocity() + VectorX<2, RealType>(-mBodyB->GetAngularVelocity() * rb.y, mBodyB->GetAngularVelocity() * rb.x) - mBodyA->GetVelocity() - VectorX<2, RealType>(-mBodyA->GetAngularVelocity() * ra.y, mBodyA->GetAngularVelocity() * ra.x);

        if (rv.lengthSquared() < (dt * gravity).lengthSquared() + MEpsilon<RealType>())
          mMixRestitution = static_cast<RealType>(0.0);
      }
    }

    void CheckContactInfo()
    {
      CollisionDispatcher<RealType>::GetInstance()->Dispatch(shared_from_this(),
                                                             mBodyA, mBodyB);
    }

  private:
    int mContactNum = 0;
    RealType mMixRestitution, mMixStaticFriction, mMixDynamicFriction;

    RealType mPenetration;
    VectorX<2, RealType> mContactDir;
    VectorX<2, RealType> mContactPoints[2];

    std::shared_ptr<RigidBody<RealType>> mBodyA, mBodyB;

    void InfiniteMassCorrection()
    {
      mBodyA->SetVelocity(VectorX<2, RealType>(static_cast<RealType>(0.0)));
      mBodyB->SetVelocity(VectorX<2, RealType>(static_cast<RealType>(0.0)));
    }
  };

} // namespace PHY::RIGIDBODY

#endif /* _COLLISION_HANDLER_H_ */