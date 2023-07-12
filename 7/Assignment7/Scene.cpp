//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Intersection p = intersect(ray);
    if(!p.happened) return Vector3f();
    if(p.obj->hasEmit()) return p.m->getEmission();

    // TO DO Implement Path Tracing Algorithm here
    Intersection inter;
    float pdf_light;
    sampleLight(inter, pdf_light);
    Vector3f _wi = inter.coords - p.coords;
    Vector3f L_dir, wi = normalize(_wi), wo = normalize(p.coords - ray.origin);
    
    // 伪代码中入射出射相反

    if((intersect(Ray(p.coords, wi)).coords - inter.coords).norm() < EPSILON) {
        L_dir = inter.emit * p.m->eval(wo, wi, p.normal) * dotProduct(wi, p.normal) * dotProduct(-wi, inter.normal)
            / pow(_wi.norm(), 2) / pdf_light;  
    }

    Vector3f L_indir = 0;

    if(get_random_float() < RussianRoulette) {
        wi = p.m->sample(wo, p.normal);
        Ray ray_new = Ray(p.coords, wi);
        Intersection q = intersect(ray_new);
        if(q.happened && !q.obj->hasEmit()) {
            L_indir = castRay(ray_new, depth + 1) * p.m->eval(wo, wi, p.normal) * dotProduct(wi, p.normal) 
                / p.m->pdf(wo, wi, p.normal) / RussianRoulette;
        }
    }

    return L_dir + L_indir;
}