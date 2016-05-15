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

Colour Scene::computeColour(const Ray &viewRay, unsigned int rayDepth) const {
    RayIntersection hitPoint = intersect(viewRay);
    if (hitPoint.distance == infinity) {
        return backgroundColour;
    }

    Material &mat = hitPoint.material;
    Normal unitNormal = hitPoint.normal / hitPoint.normal.norm();

    Colour hitColour = ambientLight * mat.ambientColour;
    Colour diffuseColour;
    Colour specularColour;

    Direction eyeDirection = -viewRay.direction;
    Direction unitEyeDirection = eyeDirection / eyeDirection.norm();
    Direction lightDirection;
    Direction unitLightDirection;
    Direction reflectedLightDirection;
    Direction unitReflectedLightDirection;

    for (auto &light : lights_) {
        lightDirection = light->location - hitPoint.point;
        unitLightDirection = lightDirection / lightDirection.norm();

        // Check if we can see this light by forming the ray p_h + t(p_l - p_h)
        // and then checking for an obstruction in 0 < t < 1
        Ray shadowRay;
        shadowRay.point = Point(hitPoint.point);
        shadowRay.direction = Direction(lightDirection);
        RayIntersection shadowIntersection = intersect(shadowRay);
        if (shadowIntersection.distance < 1) {
            // Found an obstruction, skip this light
            continue;
        }

        // Calculate diffuse colour (Lambertian) provided by this light source
        diffuseColour = mat.diffuseColour * unitLightDirection.dot(unitNormal);

        // Calculate the direction of the reflected light, r = 2n(l.n)-l
        reflectedLightDirection = 2 * unitNormal * unitLightDirection.dot(unitNormal) - unitLightDirection;
        unitReflectedLightDirection = reflectedLightDirection / reflectedLightDirection.norm();

        // Calculate specular colour (Phong) provided by this light source
        specularColour =
                mat.specularColour * std::pow(unitEyeDirection.dot(unitReflectedLightDirection), mat.specularExponent);

        // Add the diffuse and specular colours multiplied by the light source's illumination
        hitColour += light->getIntensityAt(hitPoint.point) * light->colour * (diffuseColour + specularColour);
    }

    hitColour.clip();

    return hitColour;
}

bool Scene::hasCamera() const {
    return bool(camera_);
}
