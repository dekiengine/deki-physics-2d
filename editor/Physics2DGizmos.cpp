#ifdef DEKI_EDITOR

// Colliders and joints in the Scene View: an outline of each, and on the
// selected object handles to drag its size and corners.

#include <deki-editor/Command.h>
#include <deki-editor/CommandHistory.h>
#include <deki-editor/CustomEditor.h>
#include <deki-editor/EditorRegistry.h>
#include <deki-editor/EditorUI.h>
#include <deki-editor/Localization.h>
#include <deki-editor/SceneView.h>
#include <deki/Object.h>

#include "../Physics2DComponents.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace DekiPhysics2D;

namespace DekiEditor
{
namespace
{

constexpr float kHandleRadius = 4.0f;
constexpr float kHandleHitRadius = 8.0f;
constexpr float kPi = 3.14159265f;

uint32_t OutlineColor(bool trigger, bool selected)
{
    const uint8_t alpha = selected ? 230 : 90;
    return trigger ? SceneView::Rgba(242, 180, 65, alpha) : SceneView::Rgba(92, 224, 140, alpha);
}

// The current object's axes: from metres on it, before its scale is applied,
// to the screen and back. Offsets are not scaled, as in the physics.
struct Frame
{
    float screenX, screenY, pixels, cosine, sine;

    static Frame Current()
    {
        const SceneView& view = SceneView::Get();
        return { view.GetScreenX(), view.GetScreenY(), view.GetWorldToScreenScale(), std::cos(view.GetRotation()),
                 std::sin(view.GetRotation()) };
    }
    void ToScreen(Vec2 local, float& x, float& y) const
    {
        x = screenX + (local.x * cosine - local.y * sine) * pixels;
        y = screenY - (local.x * sine + local.y * cosine) * pixels;  // the screen's Y runs down
    }
    Vec2 ToLocal(float x, float y) const
    {
        const float wx = (x - screenX) / pixels;
        const float wy = (screenY - y) / pixels;
        return { wx * cosine + wy * sine, -wx * sine + wy * cosine };
    }
};

void DrawLineLocal(const Frame& frame, Vec2 a, Vec2 b, uint32_t color)
{
    float x0, y0, x1, y1;
    frame.ToScreen(a, x0, y0);
    frame.ToScreen(b, x1, y1);
    SceneView::Get().DrawLine(x0, y0, x1, y1, color, 1.0f);
}

void DrawOutline(const Frame& frame, const Vec2* points, int count, uint32_t color)
{
    for (int i = 0; i < count; ++i)
    {
        DrawLineLocal(frame, points[i], points[(i + 1) % count], color);
    }
}

void DrawArc(const Frame& frame, Vec2 centre, float radius, float from, float sweep, uint32_t color)
{
    const int steps = 16;
    Vec2 last{ centre.x + std::cos(from) * radius, centre.y + std::sin(from) * radius };
    for (int i = 1; i <= steps; ++i)
    {
        const float angle = from + sweep * static_cast<float>(i) / steps;
        const Vec2 next{ centre.x + std::cos(angle) * radius, centre.y + std::sin(angle) * radius };
        DrawLineLocal(frame, last, next, color);
        last = next;
    }
}

// A dragged handle's edit, undone as one step. Holds the fields it changed.
class EditColliderCommand : public Command
{
public:
    struct Field
    {
        float* value;
        float before;
        float after;
    };
    EditColliderCommand(Deki::Component* component, std::vector<Field> fields)
        : m_Component(component),
          m_Fields(std::move(fields))
    {
    }
    std::string GetDescription() const override { return Tr("Edit Collider"); }
    void Execute() override
    {
        for (const Field& field : m_Fields)
        {
            *field.value = field.after;
        }
    }
    void Undo() override
    {
        for (const Field& field : m_Fields)
        {
            *field.value = field.before;
        }
    }
    bool ReferencesAny(const DeadMemory& dead) const override { return dead.Contains(m_Component); }

private:
    Deki::Component* m_Component;
    std::vector<Field> m_Fields;
};

// The one handle being dragged, across frames.
struct Drag
{
    Deki::Component* component = nullptr;
    int handle = -1;
    std::vector<EditColliderCommand::Field> fields;
};
Drag s_Drag;

/// Draws handle `id` at `local`; true while it is being dragged, with the
/// pointer in the object's axes. `fields` are the values the drag may change.
bool Handle(Deki::Component* component, const Frame& frame, int id, Vec2 local, const std::vector<float*>& fields,
            Vec2& pointer)
{
    auto& view = SceneView::Get();
    auto& ui = EditorUI::Get();
    float x, y;
    frame.ToScreen(local, x, y);
    float mouseX, mouseY;
    ui.GetMousePos(&mouseX, &mouseY);
    const bool dragging = s_Drag.component == component && s_Drag.handle == id;
    const float dx = mouseX - x;
    const float dy = mouseY - y;
    const bool hovered = view.IsPointerOnCanvas() && dx * dx + dy * dy <= kHandleHitRadius * kHandleHitRadius;

    view.DrawCircleFilled(x, y, kHandleRadius, hovered || dragging ? SceneView::Rgba(255, 255, 255) : SceneView::Rgba(92, 224, 140));
    if (hovered || dragging)
    {
        view.ClaimMouse();
    }
    if (hovered && s_Drag.component == nullptr && ui.IsMouseClicked(0))
    {
        s_Drag.component = component;
        s_Drag.handle = id;
        s_Drag.fields.clear();
        for (float* field : fields)
        {
            s_Drag.fields.push_back({ field, *field, *field });
        }
    }
    if (s_Drag.component != component || s_Drag.handle != id)
    {
        return false;
    }

    if (ui.IsMouseDown(0))
    {
        pointer = frame.ToLocal(mouseX, mouseY);
        return true;
    }

    // Let go: one undo step, if anything changed. The fields already hold the
    // new values, so Execute only repeats them.
    bool changed = false;
    for (EditColliderCommand::Field& field : s_Drag.fields)
    {
        field.after = *field.value;
        changed = changed || field.after != field.before;
    }
    if (changed)
    {
        CommandHistory::Instance().ExecuteNoMerge(std::make_unique<EditColliderCommand>(component, std::move(s_Drag.fields)));
    }
    s_Drag = {};
    return false;
}

class BoxColliderEditor : public CustomEditor
{
public:
    const char* GetComponentName() const override { return "BoxCollider2D"; }

    void OnDrawGizmos(Deki::Component* comp) override
    {
        auto* box = static_cast<BoxCollider2D*>(comp);
        auto& view = SceneView::Get();
        const Frame frame = Frame::Current();
        const float hw = box->width * 0.5f * view.GetScaleX();
        const float hh = box->height * 0.5f * view.GetScaleY();
        const Vec2 c{ box->offsetX, box->offsetY };
        const Vec2 corners[4] = {
            { c.x - hw, c.y - hh }, { c.x + hw, c.y - hh }, { c.x + hw, c.y + hh }, { c.x - hw, c.y + hh }
        };
        DrawOutline(frame, corners, 4, OutlineColor(box->isTrigger, view.IsCurrentObjectSelected()));
    }

    void OnDrawGizmosSelected(Deki::Component* comp) override
    {
        auto* box = static_cast<BoxCollider2D*>(comp);
        auto& view = SceneView::Get();
        const Frame frame = Frame::Current();
        const float scaleX = view.GetScaleX() != 0.0f ? view.GetScaleX() : 1.0f;
        const float scaleY = view.GetScaleY() != 0.0f ? view.GetScaleY() : 1.0f;
        const float hw = box->width * 0.5f * scaleX;
        const float hh = box->height * 0.5f * scaleY;
        const Vec2 c{ box->offsetX, box->offsetY };

        // Each handle moves its own side; the opposite side stays where it is.
        Vec2 pointer;
        const std::vector<float*> widthFields{ &box->width, &box->offsetX };
        const std::vector<float*> heightFields{ &box->height, &box->offsetY };
        auto horizontal = [&](float fixed)
        {
            box->offsetX = (fixed + pointer.x) * 0.5f;
            box->width = std::fabs(pointer.x - fixed) / std::fabs(scaleX);
        };
        auto vertical = [&](float fixed)
        {
            box->offsetY = (fixed + pointer.y) * 0.5f;
            box->height = std::fabs(pointer.y - fixed) / std::fabs(scaleY);
        };
        if (Handle(comp, frame, 0, { c.x + hw, c.y }, widthFields, pointer))
        {
            horizontal(c.x - hw);
        }
        if (Handle(comp, frame, 1, { c.x - hw, c.y }, widthFields, pointer))
        {
            horizontal(c.x + hw);
        }
        if (Handle(comp, frame, 2, { c.x, c.y + hh }, heightFields, pointer))
        {
            vertical(c.y - hh);
        }
        if (Handle(comp, frame, 3, { c.x, c.y - hh }, heightFields, pointer))
        {
            vertical(c.y + hh);
        }
    }
};
REGISTER_EDITOR(BoxColliderEditor)

class CircleColliderEditor : public CustomEditor
{
public:
    const char* GetComponentName() const override { return "CircleCollider2D"; }

    void OnDrawGizmos(Deki::Component* comp) override
    {
        auto* circle = static_cast<CircleCollider2D*>(comp);
        auto& view = SceneView::Get();
        const float radius = circle->radius * std::max(view.GetScaleX(), view.GetScaleY());
        DrawArc(Frame::Current(), { circle->offsetX, circle->offsetY }, radius, 0.0f, 2.0f * kPi,
                OutlineColor(circle->isTrigger, view.IsCurrentObjectSelected()));
    }

    void OnDrawGizmosSelected(Deki::Component* comp) override
    {
        auto* circle = static_cast<CircleCollider2D*>(comp);
        auto& view = SceneView::Get();
        const Frame frame = Frame::Current();
        const float scale = std::max(view.GetScaleX(), view.GetScaleY());
        const Vec2 c{ circle->offsetX, circle->offsetY };
        Vec2 pointer;
        if (Handle(comp, frame, 0, { c.x + circle->radius * scale, c.y }, { &circle->radius }, pointer) && scale > 0.0f)
        {
            circle->radius = std::hypot(pointer.x - c.x, pointer.y - c.y) / scale;
        }
    }
};
REGISTER_EDITOR(CircleColliderEditor)

class CapsuleColliderEditor : public CustomEditor
{
public:
    const char* GetComponentName() const override { return "CapsuleCollider2D"; }

    void OnDrawGizmos(Deki::Component* comp) override
    {
        auto* capsule = static_cast<CapsuleCollider2D*>(comp);
        auto& view = SceneView::Get();
        const Frame frame = Frame::Current();
        const bool vertical = capsule->direction == CapsuleDirection::Vertical;
        const float along = vertical ? view.GetScaleY() : view.GetScaleX();
        const float across = vertical ? view.GetScaleX() : view.GetScaleY();
        const float radius = capsule->radius * across;
        const float half = std::max(capsule->height * 0.5f * along - radius, 0.0f);
        const uint32_t color = OutlineColor(capsule->isTrigger, view.IsCurrentObjectSelected());
        const Vec2 c{ capsule->offsetX, capsule->offsetY };
        const Vec2 axis = vertical ? Vec2{ 0.0f, half } : Vec2{ half, 0.0f };
        const Vec2 side = vertical ? Vec2{ radius, 0.0f } : Vec2{ 0.0f, radius };
        const Vec2 top{ c.x + axis.x, c.y + axis.y };
        const Vec2 bottom{ c.x - axis.x, c.y - axis.y };
        DrawLineLocal(frame, { top.x + side.x, top.y + side.y }, { bottom.x + side.x, bottom.y + side.y }, color);
        DrawLineLocal(frame, { top.x - side.x, top.y - side.y }, { bottom.x - side.x, bottom.y - side.y }, color);
        const float start = vertical ? 0.0f : -kPi * 0.5f;
        DrawArc(frame, top, radius, start, kPi, color);
        DrawArc(frame, bottom, radius, start + kPi, kPi, color);
    }

    void OnDrawGizmosSelected(Deki::Component* comp) override
    {
        auto* capsule = static_cast<CapsuleCollider2D*>(comp);
        auto& view = SceneView::Get();
        const Frame frame = Frame::Current();
        const bool vertical = capsule->direction == CapsuleDirection::Vertical;
        const float along = vertical ? view.GetScaleY() : view.GetScaleX();
        const float across = vertical ? view.GetScaleX() : view.GetScaleY();
        const Vec2 c{ capsule->offsetX, capsule->offsetY };
        const float reach = capsule->height * 0.5f * along;
        const float radius = capsule->radius * across;

        Vec2 pointer;
        const Vec2 endAt = vertical ? Vec2{ c.x, c.y + reach } : Vec2{ c.x + reach, c.y };
        if (Handle(comp, frame, 0, endAt, { &capsule->height }, pointer) && along > 0.0f)
        {
            capsule->height = 2.0f * std::fabs(vertical ? pointer.y - c.y : pointer.x - c.x) / along;
        }
        const Vec2 sideAt = vertical ? Vec2{ c.x + radius, c.y } : Vec2{ c.x, c.y + radius };
        if (Handle(comp, frame, 1, sideAt, { &capsule->radius }, pointer) && across > 0.0f)
        {
            capsule->radius = std::fabs(vertical ? pointer.x - c.x : pointer.y - c.y) / across;
        }
    }
};
REGISTER_EDITOR(CapsuleColliderEditor)

class PolygonColliderEditor : public CustomEditor
{
public:
    const char* GetComponentName() const override { return "PolygonCollider2D"; }

    void OnDrawGizmos(Deki::Component* comp) override
    {
        auto* polygon = static_cast<PolygonCollider2D*>(comp);
        auto& view = SceneView::Get();
        // The outline the physics uses, which fills in any dent.
        Vec2 points[kMaxPolygonPoints];
        Vec2 outline[kMaxPolygonPoints];
        const int count = std::clamp<int32_t>(polygon->pointCount, 0, kMaxPolygonPoints);
        for (int i = 0; i < count; ++i)
        {
            points[i] = { polygon->offsetX + polygon->Point(i).x * view.GetScaleX(),
                          polygon->offsetY + polygon->Point(i).y * view.GetScaleY() };
        }
        const int corners = count >= 3 ? ConvexOutline(points, count, outline) : 0;
        DrawOutline(Frame::Current(), outline, corners, OutlineColor(polygon->isTrigger, view.IsCurrentObjectSelected()));
    }

    void OnDrawGizmosSelected(Deki::Component* comp) override
    {
        auto* polygon = static_cast<PolygonCollider2D*>(comp);
        auto& view = SceneView::Get();
        const Frame frame = Frame::Current();
        const float scaleX = view.GetScaleX() != 0.0f ? view.GetScaleX() : 1.0f;
        const float scaleY = view.GetScaleY() != 0.0f ? view.GetScaleY() : 1.0f;
        const int count = std::clamp<int32_t>(polygon->pointCount, 0, kMaxPolygonPoints);
        for (int i = 0; i < count; ++i)
        {
            Deki::Vector2& point = polygon->Point(i);
            Vec2 pointer;
            if (Handle(comp, frame, i, { polygon->offsetX + point.x * scaleX, polygon->offsetY + point.y * scaleY },
                       { &point.x, &point.y }, pointer))
            {
                point.x = (pointer.x - polygon->offsetX) / scaleX;
                point.y = (pointer.y - polygon->offsetY) / scaleY;
            }
        }
    }
};
REGISTER_EDITOR(PolygonColliderEditor)

void DrawPin(float x, float y, uint32_t color)
{
    SceneView::Get().DrawCircle(x, y, 5.0f, color, 1.5f);
}

// A point on another object, on the screen of the object being drawn.
void OtherObjectToScreen(const Frame& frame, const Deki::Object* current, const Deki::Object* other, float& x, float& y)
{
    const float wx = other->GetWorldX() - current->GetWorldX();
    const float wy = other->GetWorldY() - current->GetWorldY();
    x = frame.screenX + wx * frame.pixels;
    y = frame.screenY - wy * frame.pixels;
}

class HingeJointEditor : public CustomEditor
{
public:
    const char* GetComponentName() const override { return "HingeJoint2D"; }

    void OnDrawGizmos(Deki::Component* comp) override
    {
        auto* joint = static_cast<HingeJoint2D*>(comp);
        auto& view = SceneView::Get();
        const Frame frame = Frame::Current();
        const uint32_t color = SceneView::Rgba(76, 166, 232, view.IsCurrentObjectSelected() ? 230 : 110);
        float x, y;
        frame.ToScreen({ joint->anchorX * view.GetScaleX(), joint->anchorY * view.GetScaleY() }, x, y);
        DrawPin(x, y, color);
        view.DrawLine(view.GetScreenX(), view.GetScreenY(), x, y, color, 1.0f);
        if (Rigidbody2D* other = joint->connectedBody.Get())
        {
            float ox, oy;
            OtherObjectToScreen(frame, view.GetCurrentObject(), other->GetOwner(), ox, oy);
            view.DrawLine(x, y, ox, oy, color, 1.0f);
        }
    }
};
REGISTER_EDITOR(HingeJointEditor)

class DistanceJointEditor : public CustomEditor
{
public:
    const char* GetComponentName() const override { return "DistanceJoint2D"; }

    void OnDrawGizmos(Deki::Component* comp) override
    {
        auto* joint = static_cast<DistanceJoint2D*>(comp);
        auto& view = SceneView::Get();
        const Frame frame = Frame::Current();
        const uint32_t color = SceneView::Rgba(76, 166, 232, view.IsCurrentObjectSelected() ? 230 : 110);
        float x, y;
        frame.ToScreen({ joint->anchorX * view.GetScaleX(), joint->anchorY * view.GetScaleY() }, x, y);
        DrawPin(x, y, color);
        float ox, oy;
        if (Rigidbody2D* other = joint->connectedBody.Get())
        {
            OtherObjectToScreen(frame, view.GetCurrentObject(), other->GetOwner(), ox, oy);
        }
        else
        {
            const Deki::Object* object = view.GetCurrentObject();
            ox = frame.screenX + (joint->connectedAnchorX - object->GetWorldX()) * frame.pixels;
            oy = frame.screenY - (joint->connectedAnchorY - object->GetWorldY()) * frame.pixels;
        }
        DrawPin(ox, oy, color);
        view.DrawLine(x, y, ox, oy, color, 1.0f);
    }
};
REGISTER_EDITOR(DistanceJointEditor)

}  // namespace
}  // namespace DekiEditor

#endif  // DEKI_EDITOR
