#ifndef _CPP_FACE_FACEMARK_H_
#define _CPP_FACE_FACEMARK_H_

// ReSharper disable CppNonInlineFunctionDefinitionInHeaderFile

#include "include_opencv.h"


namespace cv
{
    namespace face
    {
        class CV_EXPORTS_W Facemark;
        class CV_EXPORTS_W FaceRecognizer;
        class CV_EXPORTS_W EigenFaceRecognizer;
        class CV_EXPORTS_W FisherFaceRecognizer;
        class CV_EXPORTS_W LBPHFaceRecognizer;
#ifndef HAVE_OPENCV_FACE
        class CV_EXPORTS_W FacemarkLBF
        {
        public:
            struct CV_EXPORTS Params;
        };
        class CV_EXPORTS_W FacemarkAAM
        {
        public:
            struct CV_EXPORTS Params;
        };
#endif//HAVE_OPENCV_FACE
    }
}

#pragma region Facemark

CVAPI(ExceptionStatus) face_Facemark_loadModel(cv::face::Facemark *obj, const char *model)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->loadModel(model);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus)
face_Facemark_fit(
    cv::face::Facemark *obj,
    cv::_InputArray *image,
    cv::_InputArray *faces,
    std::vector<std::vector<cv::Point2f>> *landmarks,
    int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->fit(*image, *faces, *landmarks) ? 1 : 0;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

#pragma endregion

#pragma region FacemarkLBF

CVAPI(ExceptionStatus) face_FacemarkLBF_create(cv::face::FacemarkLBF::Params *params, cv::Ptr<cv::face::FacemarkLBF> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto obj = (params == nullptr) ? 
        cv::face::FacemarkLBF::create() :
        cv::face::FacemarkLBF::create(*params);
    *returnValue = clone(obj);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_Ptr_FacemarkLBF_get(cv::Ptr<cv::face::FacemarkLBF> *obj, cv::face::FacemarkLBF **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->get();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_Ptr_FacemarkLBF_delete(cv::Ptr<cv::face::FacemarkLBF> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    delete obj;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

#pragma region Params

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_new(cv::face::FacemarkLBF::Params **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = new cv::face::FacemarkLBF::Params;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_delete(cv::face::FacemarkLBF::Params *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    delete obj;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_shape_offset_get(cv::face::FacemarkLBF::Params *obj, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->shape_offset;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_shape_offset_set(cv::face::FacemarkLBF::Params *obj, double val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->shape_offset = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_cascade_face_get(cv::face::FacemarkLBF::Params *obj, std::string *s)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    s->assign(obj->cascade_face);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_cascade_face_set(cv::face::FacemarkLBF::Params *obj, const char *s)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
     obj->cascade_face = s;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_verbose_get(cv::face::FacemarkLBF::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->verbose ? 1 : 0;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_verbose_set(cv::face::FacemarkLBF::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->verbose = (val != 0);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_n_landmarks_get(cv::face::FacemarkLBF::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->n_landmarks;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_n_landmarks_set(cv::face::FacemarkLBF::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->n_landmarks = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_initShape_n_get(cv::face::FacemarkLBF::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->initShape_n;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_initShape_n_set(cv::face::FacemarkLBF::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->initShape_n = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_stages_n_get(cv::face::FacemarkLBF::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->stages_n;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_stages_n_set(cv::face::FacemarkLBF::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->stages_n = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_tree_n_get(cv::face::FacemarkLBF::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->tree_n;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_tree_n_set(cv::face::FacemarkLBF::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->tree_n = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_tree_depth_get(cv::face::FacemarkLBF::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->tree_depth;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_tree_depth_set(cv::face::FacemarkLBF::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->tree_depth = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_bagging_overlap_get(cv::face::FacemarkLBF::Params *obj, double *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->bagging_overlap;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_bagging_overlap_set(cv::face::FacemarkLBF::Params *obj, double val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->bagging_overlap = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_model_filename_get(cv::face::FacemarkLBF::Params *obj, std::string *s)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    s->assign(obj->model_filename);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_model_filename_set(cv::face::FacemarkLBF::Params *obj, const char *s)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->model_filename = s;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_save_model_get(cv::face::FacemarkLBF::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->save_model ? 1 : 0;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_save_model_set(cv::face::FacemarkLBF::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->save_model = (val != 0);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_seed_get(cv::face::FacemarkLBF::Params *obj, unsigned int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->seed;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_seed_set(cv::face::FacemarkLBF::Params *obj, unsigned int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->seed = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_feats_m_get(cv::face::FacemarkLBF::Params *obj, std::vector<int> *v)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    std::copy(obj->feats_m.begin(), obj->feats_m.end(), std::back_inserter(*v));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_feats_m_set(cv::face::FacemarkLBF::Params *obj, std::vector<int> *v)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->feats_m.clear();
    std::copy(v->begin(), v->end(), std::back_inserter(obj->feats_m));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_radius_m_get(cv::face::FacemarkLBF::Params *obj, std::vector<double> *v)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    std::copy(obj->radius_m.begin(), obj->radius_m.end(), std::back_inserter(*v));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_radius_m_set(cv::face::FacemarkLBF::Params *obj, std::vector<double> *v)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->radius_m.clear();
    std::copy(v->begin(), v->end(), std::back_inserter(obj->radius_m));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_pupils0_get(cv::face::FacemarkLBF::Params *obj, std::vector<int> *v)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    std::copy(obj->pupils[0].begin(), obj->pupils[0].end(), std::back_inserter(*v));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_pupils0_set(cv::face::FacemarkLBF::Params *obj, std::vector<int> *v)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->pupils[0].clear();
    std::copy(v->begin(), v->end(), std::back_inserter(obj->pupils[0]));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_pupils1_get(cv::face::FacemarkLBF::Params *obj, std::vector<int> *v)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    std::copy(obj->pupils[1].begin(), obj->pupils[1].end(), std::back_inserter(*v));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_pupils1_set(cv::face::FacemarkLBF::Params *obj, std::vector<int> *v)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->pupils[1].clear();
    std::copy(v->begin(), v->end(), std::back_inserter(obj->pupils[1]));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_detectROI_get(cv::face::FacemarkLBF::Params *obj, MyCvRect *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = c(obj->detectROI);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkLBF_Params_detectROI_set(cv::face::FacemarkLBF::Params *obj, MyCvRect val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->detectROI = cpp(val);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}


CVAPI(ExceptionStatus) face_FacemarkLBF_Params_read(cv::face::FacemarkLBF::Params *obj, cv::FileNode *fn)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->read(*fn);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkLBF_Params_write(cv::face::FacemarkLBF::Params *obj, cv::FileStorage *fs)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->write(*fs);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

#pragma endregion
#pragma endregion

#pragma region FacemarkAAM

CVAPI(ExceptionStatus) face_FacemarkAAM_create(cv::face::FacemarkAAM::Params *params, cv::Ptr<cv::face::FacemarkAAM> **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    const auto obj = (params == nullptr) ?
        cv::face::FacemarkAAM::create() :
        cv::face::FacemarkAAM::create(*params);
    *returnValue = clone(obj);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_Ptr_FacemarkAAM_get(cv::Ptr<cv::face::FacemarkAAM> *obj, cv::face::FacemarkAAM **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->get();
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_Ptr_FacemarkAAM_delete(cv::Ptr<cv::face::FacemarkAAM> *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    delete obj;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

#pragma region Params

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_new(cv::face::FacemarkAAM::Params **returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = new cv::face::FacemarkAAM::Params;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_delete(cv::face::FacemarkAAM::Params *obj)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    delete obj;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_model_filename_get(cv::face::FacemarkAAM::Params *obj, std::string *s)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    s->assign(obj->model_filename);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkAAM_Params_model_filename_set(cv::face::FacemarkAAM::Params *obj, const char *s)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->model_filename = s;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_m_get(cv::face::FacemarkAAM::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->m;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkAAM_Params_m_set(cv::face::FacemarkAAM::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->m = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_n_get(cv::face::FacemarkAAM::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->n;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkAAM_Params_n_set(cv::face::FacemarkAAM::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->n = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_n_iter_get(cv::face::FacemarkAAM::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->n_iter;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkAAM_Params_n_iter_set(cv::face::FacemarkAAM::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->n_iter = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_verbose_get(cv::face::FacemarkAAM::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->verbose ? 1 : 0;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkAAM_Params_verbose_set(cv::face::FacemarkAAM::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->verbose = (val != 0);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_save_model_get(cv::face::FacemarkAAM::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->save_model ? 1 : 0;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkAAM_Params_save_model_set(cv::face::FacemarkAAM::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->save_model = (val != 0);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_max_m_get(cv::face::FacemarkAAM::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->max_m;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkAAM_Params_max_m_set(cv::face::FacemarkAAM::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->max_m = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_max_n_get(cv::face::FacemarkAAM::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->max_n;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkAAM_Params_max_n_set(cv::face::FacemarkAAM::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->max_n = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_texture_max_m_get(cv::face::FacemarkAAM::Params *obj, int *returnValue)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    *returnValue = obj->texture_max_m;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkAAM_Params_texture_max_m_set(cv::face::FacemarkAAM::Params *obj, int val)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->texture_max_m = val;
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_scales_get(cv::face::FacemarkAAM::Params *obj, std::vector<float> *v)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    std::copy(obj->scales.begin(), obj->scales.end(), std::back_inserter(*v));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}
CVAPI(ExceptionStatus) face_FacemarkAAM_Params_scales_set(cv::face::FacemarkAAM::Params *obj, std::vector<float> *v)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->scales.clear();
    std::copy(v->begin(), v->end(), std::back_inserter(obj->scales));
#endif//HAVE_OPENCV_FACE
    END_WRAP
}


CVAPI(ExceptionStatus) face_FacemarkAAM_Params_read(cv::face::FacemarkAAM::Params *obj, cv::FileNode *fn)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->read(*fn);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

CVAPI(ExceptionStatus) face_FacemarkAAM_Params_write(cv::face::FacemarkAAM::Params *obj, cv::FileStorage *fs)
{
    BEGIN_WRAP
#ifdef HAVE_OPENCV_FACE
    obj->write(*fs);
#endif//HAVE_OPENCV_FACE
    END_WRAP
}

#pragma endregion
#pragma endregion

#endif