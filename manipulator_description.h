#pragma once

//генерируемый файл, содержащий описание геометрии манипулятора

//создание массива деталей
const unsigned int partNumber = 4;
static Part* parts[partNumber];
ArrayView<Part*> partsProtected(parts, partNumber);
m_parts = partsProtected;

const unsigned int pairMayCollideNumber = 1;
static PairOfPartsMayCollide pairMayCollideArray[pairMayCollideNumber];
ArrayView<PairOfPartsMayCollide> pairMayCollideArrayProtected(pairMayCollideArray, pairMayCollideNumber);
m_pairsOfPartsForChecking = pairMayCollideArrayProtected;

//описание детали base

const unsigned int base_boxCount = 0;
static OBB_GJK* base_boxArray = nullptr;
ArrayView<OBB_GJK> base_boxArrayProtected(base_boxArray, base_boxCount);

const unsigned int base_sphereCount = 0;
static Sphere_GJK* base_sphereArray = nullptr;
ArrayView<Sphere_GJK> base_sphereArrayProtected(base_sphereArray, base_sphereCount);

const unsigned int base_cylinderCount = 0;
static Cylinder_GJK* base_cylinderArray = nullptr;
ArrayView<Cylinder_GJK> base_cylinderArrayProtected(base_cylinderArray, base_cylinderCount);

static Part base(Vector3f(0, 0, 0), Vector3f::UnitX(), NULL, true, NAN, NAN);
base.setRoughBounding(Vector3f(0, 0, 0), Vector3f(0, 0, 0), Vector3f(0, 0, 0));

base.setBoxArray(base_boxArrayProtected);
base.setSphereArray(base_sphereArrayProtected);
base.setCylinderArray(base_cylinderArrayProtected);
partsProtected[0] = &base;

//описание детали part1

const unsigned int part1_boxCount = 0;
static OBB_GJK* part1_boxArray = nullptr;
ArrayView<OBB_GJK> part1_boxArrayProtected(part1_boxArray, part1_boxCount);

const unsigned int part1_sphereCount = 0;
static Sphere_GJK* part1_sphereArray = nullptr;
ArrayView<Sphere_GJK> part1_sphereArrayProtected(part1_sphereArray, part1_sphereCount);

const unsigned int part1_cylinderCount = 0;
static Cylinder_GJK* part1_cylinderArray = nullptr;
ArrayView<Cylinder_GJK> part1_cylinderArrayProtected(part1_cylinderArray, part1_cylinderCount);

static Part part1(Vector3f(0, 0, 0), Vector3f::UnitX(), &base, true, NAN, NAN);
part1.setRoughBounding(Vector3f(0, 0, 0), Vector3f(0, 0, 0), Vector3f(0, 0, 0));

part1.setBoxArray(part1_boxArrayProtected);
part1.setSphereArray(part1_sphereArrayProtected);
part1.setCylinderArray(part1_cylinderArrayProtected);
partsProtected[1] = &part1;

//описание детали part2

const unsigned int part2_boxCount = 2;
static OBB_GJK part2_boxArray[part2_boxCount];
ArrayView<OBB_GJK> part2_boxArrayProtected(part2_boxArray, part2_boxCount);
part2_boxArrayProtected[0] = OBB_GJK(Vector3f(0, 0, 0), Vector3f(0, 0, 0), Vector3f(0, 0, 0));
part2_boxArrayProtected[1] = OBB_GJK(Vector3f(0, 0, 0), Vector3f(0, 0, 0), Vector3f(0, 0, 0));

const unsigned int part2_sphereCount = 1;
static Sphere_GJK part2_sphereArray[part2_sphereCount];
ArrayView<Sphere_GJK> part2_sphereArrayProtected(part2_sphereArray, part2_sphereCount);
part2_sphereArrayProtected[0] = Sphere_GJK(0, Vector3f(0, 0, 0), Vector3f(0, 0, 0));

const unsigned int part2_cylinderCount = 1;
static Cylinder_GJK part2_cylinderArray[part2_cylinderCount];
ArrayView<Cylinder_GJK> part2_cylinderArrayProtected(part2_cylinderArray, part2_cylinderCount);
part2_cylinderArrayProtected[0] = Cylinder_GJK(0, 0, Vector3f(0, 0, 0), Vector3f(0, 0, 0));

static Part part2(Vector3f(0, 0, 0), Vector3f::UnitX(), &base, true, NAN, NAN);
part2.setRoughBounding(Vector3f(0, 0, 0), Vector3f(0, 0, 0), Vector3f(0, 0, 0));

part2.setBoxArray(part2_boxArrayProtected);
part2.setSphereArray(part2_sphereArrayProtected);
part2.setCylinderArray(part2_cylinderArrayProtected);
partsProtected[2] = &part2;

//описание детали obs1

const unsigned int obs1_boxCount = 0;
static OBB_GJK* obs1_boxArray = nullptr;
ArrayView<OBB_GJK> obs1_boxArrayProtected(obs1_boxArray, obs1_boxCount);

const unsigned int obs1_sphereCount = 0;
static Sphere_GJK* obs1_sphereArray = nullptr;
ArrayView<Sphere_GJK> obs1_sphereArrayProtected(obs1_sphereArray, obs1_sphereCount);

const unsigned int obs1_cylinderCount = 0;
static Cylinder_GJK* obs1_cylinderArray = nullptr;
ArrayView<Cylinder_GJK> obs1_cylinderArrayProtected(obs1_cylinderArray, obs1_cylinderCount);

static Part obs1(Vector3f(0, 0, 0), Vector3f::UnitX(), NULL, true, NAN, NAN);
obs1.setRoughBounding(Vector3f(0, 0, 0), Vector3f(0, 0, 0), Vector3f(0, 0, 0));

obs1.setBoxArray(obs1_boxArrayProtected);
obs1.setSphereArray(obs1_sphereArrayProtected);
obs1.setCylinderArray(obs1_cylinderArrayProtected);
partsProtected[3] = &obs1;

