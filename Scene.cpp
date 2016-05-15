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
    for (auto &obj: objects_) {
        auto hits = obj->intersect(ray);
    }
    return firstHit;
}

Colour Scene::computeColour(const Ray &viewRay, unsigned int rayDepth) const {
    RayIntersection hitPoint = intersect(viewRay);
    if (hitPoint.distance == infinity) {
        return backgroundColour;
    }

    Colour hitColour = ambientLight * hitPoint.material.ambientColour;

    Material &mat = hitPoint.material;
    for (auto &light : lights_) {
        // Check if we can see this light
    }

    hitColour.clip();

    return hitColour;
}

bool Scene::hasCamera() const {
    return bool(camera_);
}
