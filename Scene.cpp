/* $Rev: 250 $ */
#include "Scene.h"

#include "Colour.h"
#include "Display.h"
#include "utility.h"

Scene::Scene() : backgroundColour(0, 0, 0), ambientLight(0, 0, 0), maxRayDepth(3), renderWidth(800), renderHeight(600),
                 filename("render.png"), camera_(), objects_(), lights_() {

}

Scene::~Scene() {

}

void Scene::render() const {
    Display display("Render", renderWidth, renderHeight, Colour(128, 128, 128));

    std::cout << "Rendering a scene with " << objects_.size() << " objects" << std::endl;

    double halfPixel = 2.0 / (2 * renderWidth);

    for (unsigned int y = 0; y < renderHeight; ++y) {
        for (unsigned int x = 0; x < renderWidth; ++x) {
            double cx = (x - 0.5 * renderWidth) * 2.0 / renderWidth + halfPixel;
            double cy = (y - 0.5 * renderHeight) * 2.0 / renderWidth + halfPixel;
            Ray ray = camera_->castRay(cx, cy);
            display.set(x, y, computeColour(ray, maxRayDepth));
        }
        display.refresh();
    }
    display.save(filename);
    display.pause(5);
}

RayIntersection Scene::intersect(const Ray &ray) const {
    RayIntersection firstHit;
    firstHit.distance = infinity;
    for (auto &obj : objects_) {
        std::vector<RayIntersection> hits = obj->intersect(ray);
        for (auto &hit : hits) {
            if (hit.distance > epsilon && hit.distance < firstHit.distance) {
                firstHit = hit;
            }
        }
    }
    return firstHit;
}

bool Scene::isInShadowFromLight(const Point &point, const std::shared_ptr<LightSource> &light) const {
    Ray shadowRay;
    shadowRay.point = Point(point);
    shadowRay.direction = Direction(light->location - point);

    for (auto &obj : objects_) {
        std::vector<RayIntersection> hits = obj->intersect(shadowRay);
        for (auto &hit : hits) {
            if (hit.distance > 0.0 && hit.distance < shadowRay.direction.norm()) {
                return true;
            }
        }
    }

    return false;
}

Colour Scene::computeColour(const Ray &viewRay, unsigned int rayDepth) const {
    RayIntersection hitPoint = intersect(viewRay);
    if (hitPoint.distance == infinity) {
        return backgroundColour;
    }

    Material &mat = hitPoint.material;
    Normal unitNormal = hitPoint.normal / hitPoint.normal.norm();
    Point &point = hitPoint.point;

    Colour hitColour = ambientLight * mat.ambientColour;
    Colour diffuseColour;
    Colour specularColour;

    Direction unitEyeDirection = -viewRay.direction / viewRay.direction.norm();
    Direction unitLightDirection;
    Direction reflectedLightDirection;
    Direction unitReflectedLightDirection;

    for (auto &light : lights_) {
        unitLightDirection = light->location - point;
        unitLightDirection /= unitLightDirection.norm();

        // Check if we can see this light
        if (isInShadowFromLight(point, light))
            continue;

        // Calculate diffuse colour (Lambertian) provided by this light source
        diffuseColour = mat.diffuseColour * std::max(unitLightDirection.dot(unitNormal), 0.0);

        // Calculate the direction of the reflected light, r = 2n(l.n)-l
        reflectedLightDirection = 2 * unitNormal * unitLightDirection.dot(unitNormal) - unitLightDirection;
        unitReflectedLightDirection = reflectedLightDirection / reflectedLightDirection.norm();

        // Calculate specular colour (Phong) provided by this light source
        specularColour = mat.specularColour * std::pow(std::max(unitEyeDirection.dot(unitReflectedLightDirection), 0.0), mat.specularExponent);

        // Add the diffuse and specular colours multiplied by the light source's illumination
        hitColour += light->getIntensityAt(point) * light->colour * (diffuseColour + specularColour);
    }

    // Check for ray depth to prevent infinite recursion
    // Also check the hit materials mirror colour isn't tiny, to prevent unnecesary reflection recursion
    if (rayDepth > 0 && mat.mirrorColour.red > epsilon && mat.mirrorColour.green > epsilon && mat.mirrorColour.blue > epsilon) {
        // Cast reflection ray, r = 2n(e.n)-e
        Ray reflectionRay;
        reflectionRay.point = point;
        reflectionRay.direction = 2 * unitNormal * unitEyeDirection.dot(unitNormal) - unitEyeDirection;

        // Get the colour at to be reflected
        Colour reflectedColour = computeColour(reflectionRay, rayDepth - 1);
        hitColour += mat.mirrorColour * reflectedColour;
    }

    hitColour.clip();

    return hitColour;
}

bool Scene::hasCamera() const {
    return bool(camera_);
}
