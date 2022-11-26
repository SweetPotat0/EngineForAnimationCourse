#include "BasicScene.h"
#include "igl/edge_flaps.h"
#include "igl/read_triangle_mesh.h"
#include "igl/min_heap.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/collapse_edge.h"


using namespace cg3d;

Eigen::MatrixXi OF, F, E, EF, EI;
Eigen::MatrixXd V, OV;
Eigen::VectorXi EMAP;
igl::min_heap<std::tuple<double, int, int>> Q;

// I think: Time vector for each collapse
Eigen::VectorXi EQ;
// If an edge were collapsed, we'd collapse it to these points:
Eigen::MatrixXd C;
int num_collapsed;
std::shared_ptr<Model> cube;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create("camera", fov, float(width) / float(height), near, far);
    std::shared_ptr<cg3d::Program> program = std::make_shared<Program>("shaders/basicShader");
    std::shared_ptr<cg3d::Material> material = std::make_shared<Material>("material", program); // empty material
    auto cubeMesh{IglLoader::MeshFromFiles("cube_igl","data/cube.off")};

    cube = Model::Create( "cube", cubeMesh, material);
    AddChild(cube);

    camera->Translate(15, Axis::Z);
    cube->Scale(3);

    OV = cube->GetMeshList()[0]->data[0].vertices;
    OF = cube->GetMeshList()[0]->data[0].faces;
    // igl::read_triangle_mesh("data/cube.off", OV, OF);

    // Function to reset original mesh and data structures
    F = OF;
    V = OV;
    igl::edge_flaps(F, E, EMAP, EF, EI);
    C.resize(E.rows(), V.cols());
    Eigen::VectorXd costs(E.rows());
    Q = {};
    EQ = Eigen::VectorXi::Zero(E.rows());
    {
        Eigen::VectorXd costs(E.rows());
        igl::parallel_for(
            E.rows(), [&](const int e)
            {
                double cost = e;
                Eigen::RowVectorXd p(1,3);
                igl::shortest_edge_and_midpoint(e,V,F,E,EMAP,EF,EI,cost,p);
                C.row(e) = p;
                costs(e) = cost; },
            10000);
        for (int e = 0; e < E.rows(); e++)
        {
            Q.emplace(costs(e), e, 0);
        }
    }

    num_collapsed = 0;
    // viewer.data().clear();
    // viewer.data().set_mesh(V, F);
    // viewer.data().set_face_based(true);

    // igl::edge_flaps(OF, E, EMAP, EF, EI);
    // std::cout << "vertices" << std::endl;
    // std::cout << OV.transpose() << std::endl;
    // std::cout << "faces: " << std::endl;
    // std::cout << OF.transpose() << std::endl;
    // std::cout << "E - edges: " << std::endl;
    // std::cout << E.transpose() << std::endl;
    // std::cout << "EMAP - faces to edges: " << std::endl;
    // std::cout << EMAP.transpose() << std::endl;
    // std::cout << "EF - edges to faces: " << std::endl;
    // std::cout << EF.transpose() << std::endl;
    // std::cout << "EI - edge to vertex index in triangle (0,1,2): " << std::endl;
    // std::cout << EI.transpose() << std::endl;
}

bool collapseTenPerEdges()
{
    // If animating then collapse 10% of edges
    if (!Q.empty())
    {
        std::cout << "(" << std::get<0>(Q.top()) << "," << std::get<1>(Q.top()) << ")" << std::endl;
        bool something_collapsed = false;
        // collapse edge
        const int max_iter = std::ceil(0.01 * Q.size());
        for (int j = 0; j < max_iter; j++)
        {
            if (!igl::collapse_edge(igl::shortest_edge_and_midpoint, V, F, E, EMAP, EF, EI, Q, EQ, C))
            {
                break;
            }
            something_collapsed = true;
            num_collapsed++;
        }
        std::vector<std::shared_ptr<Mesh>> newlist = std::vector<std::shared_ptr<Mesh>>();
        
        Eigen::MatrixXd normals;
        auto oldList = cube->GetMeshList();
        auto meshCount = oldList.size();
        auto meshData = cube->GetMesh()->data[0];
        auto moshe = meshData.vertices;
        auto v = moshe;
        for (size_t i = 0; i < v.rows(); i++)
        {
            Eigen::RowVectorXd moshe = cube->GetMesh()->data[0].vertexNormals.row(i).normalized();
            normals(i) = moshe[0];
            normals(i+1) = moshe[1];
            normals(i+2) = moshe[2];
        }
        Eigen::MatrixXd texCoords = cube->GetMesh()->data[0].textureCoords;
        newlist.push_back(std::shared_ptr<Mesh>{new Mesh("moshe", V, F, normals, texCoords)});
        cube->SetMeshList(newlist);

        // if (something_collapsed)
        // {
        //     viewer.data().clear();
        //     viewer.data().set_mesh(V, F);
        //     viewer.data().set_face_based(true);
        // }
    }
    return false;
}

int frameCount = 0;

void BasicScene::Update(const Program &program, const Eigen::Matrix4f &proj, const Eigen::Matrix4f &view, const Eigen::Matrix4f &model)
{
    frameCount++;
    Scene::Update(program, proj, view, model);

    cube->Rotate(0.01f, Axis::XYZ);

    if (frameCount == 500)
    {
        std::cout << "collapsing:" << std::endl;
        collapseTenPerEdges();
    }
}
