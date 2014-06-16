

#include <assert.h>

#include "Mesh.hpp"

std::vector<unsigned int> Indices;



Mesh::MeshEntry::MeshEntry()
{
	VB = 0xffffffff;
	IB = 0xffffffff;
	NumIndices  = 0;
	MaterialIndex = INVALID_MATERIAL;
};

Mesh::MeshEntry::~MeshEntry()
{
	if (VB != 0xffffffff)
	{
		glDeleteBuffers(1, &VB);
	}

	if (IB != 0xffffffff)
	{
		glDeleteBuffers(1, &IB);
	}
}

void Mesh::MeshEntry::Init(const std::vector<Vertex>& Vertices,
		const std::vector<unsigned int>& Indices)
{
	NumIndices = Indices.size();

	glGenBuffers(1, &VB);
	glBindBuffer(GL_ARRAY_BUFFER, VB);

	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * Vertices.size(), &Vertices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &IB);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IB);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * NumIndices, &Indices[0], GL_STATIC_DRAW);
}

Mesh::Mesh()
{
	color[0] = 0.0;
	color[1] = 0.0;
	color[2] = 0.0;
}

void Mesh::setColor(float r, float g, float b){
	color[0] = r;
	color[1] = g;
	color[2] = b;
}

Mesh::~Mesh()
{
	Clear();
}


void Mesh::Clear()
{
}


// focntion princiapele qui load une mesh
bool Mesh::LoadMesh(const std::string& Filename)
{
	// Release the previously loaded mesh (if it exists)
	Clear();

	bool Ret = false;
	Assimp::Importer Importer;

	const aiScene* pScene = Importer.ReadFile(Filename.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs);

	if (pScene) {
		Ret = InitFromScene(pScene, Filename);
	}
	else {
		printf("Error parsing '%s': '%s'\n", Filename.c_str(), Importer.GetErrorString());
	}

	return Ret;
}

bool Mesh::InitFromScene(const aiScene* pScene, const std::string& Filename)
{  
	m_Entries.resize(pScene->mNumMeshes);
	// m_Textures.resize(pScene->mNumMaterials);

	// Initialize the meshes in the scene one by one
	for (unsigned int i = 0 ; i < m_Entries.size() ; i++) {
		const aiMesh* paiMesh = pScene->mMeshes[i];
		InitMesh(i, paiMesh);
	}

	return InitMaterials(pScene, Filename);
}

void Mesh::InitMesh(unsigned int Index, const aiMesh* paiMesh)
{
	m_Entries[Index].MaterialIndex = paiMesh->mMaterialIndex;

	//std::vector<Vertex> Vertices;
	//std::vector<unsigned int> Indices;

	const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);

	for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
		const aiVector3D* pPos      = &(paiMesh->mVertices[i]);
		const aiVector3D* pNormal   = &(paiMesh->mNormals[i]);
		const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;

		Vertex v(Vector3f(pPos->x, pPos->z, pPos->y),
				Vector2f(pTexCoord->x, pTexCoord->y),
				Vector3f(pNormal->x, pNormal->z, pNormal->y));

		Vertices.push_back(v);
	}

	for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
		const aiFace& Face = paiMesh->mFaces[i];
		assert(Face.mNumIndices == 3);
		Indices.push_back(Face.mIndices[0]);
		Indices.push_back(Face.mIndices[1]);
		Indices.push_back(Face.mIndices[2]);
	}

	m_Entries[Index].Init(Vertices, Indices);
}

bool Mesh::InitMaterials(const aiScene* pScene, const std::string& Filename)
{
	// Extract the directory part from the file name
	std::string::size_type SlashIndex = Filename.find_last_of("/");
	std::string Dir;

	if (SlashIndex == std::string::npos) {
		Dir = ".";
	}
	else if (SlashIndex == 0) {
		Dir = "/";
	}
	else {
		Dir = Filename.substr(0, SlashIndex);
	}

	bool Ret = true;
	/*
	// Initialize the materials
	for (unsigned int i = 0 ; i < pScene->mNumMaterials ; i++) {
	const aiMaterial* pMaterial = pScene->mMaterials[i];

	m_Textures[i] = NULL;

	if (pMaterial->GetTextureCount(aiTextureType_DIFFUSE) > 0) {
	aiString Path;

	if (pMaterial->GetTexture(aiTextureType_DIFFUSE, 0, &Path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS) {
	std::string FullPath = Dir + "/" + Path.data;
	m_Textures[i] = new Texture(GL_TEXTURE_2D, FullPath.c_str());

	if (!m_Textures[i]->Load()) {
	printf("Error loading texture '%s'\n", FullPath.c_str());
	delete m_Textures[i];
	m_Textures[i] = NULL;
	Ret = false;
	}
	else {
	printf("Loaded texture '%s'\n", FullPath.c_str());
	}
	}
	}

	// Load a white texture in case the model does not include its own texture
	if (!m_Textures[i]) {
	m_Textures[i] = new Texture(GL_TEXTURE_2D, "./white.png");

	Ret = m_Textures[i]->Load();
	}
	}
	 */

	return Ret;
}

void Mesh::Render(float ratio)
{
	glColor3f(color[0], color[1], color[2]);
	for(int i=0; i<Vertices.size(); i+=3){
			glBegin(GL_TRIANGLES); //Begin triangle coordinates
			glNormal3f((Vertices[i]).m_normal.x,(Vertices[i]).m_normal.y,(Vertices[i]).m_normal.z);

			glVertex3f((Vertices[i]).m_pos.x*ratio,(Vertices[i]).m_pos.y*ratio,(Vertices[i]).m_pos.z*ratio);

			glNormal3f((Vertices[i+1]).m_normal.x,(Vertices[i+1]).m_normal.y,(Vertices[i+1]).m_normal.z);
			glVertex3f((Vertices[i+1]).m_pos.x*ratio,(Vertices[i+1]).m_pos.y*ratio,(Vertices[i+1]).m_pos.z*ratio);

			glNormal3f((Vertices[i+2]).m_normal.x,(Vertices[i+2]).m_normal.y,(Vertices[i+2]).m_normal.z);
			glVertex3f((Vertices[i+2]).m_pos.x*ratio,(Vertices[i+2]).m_pos.y*ratio,(Vertices[i+2]).m_pos.z*ratio);
			glEnd(); //End triangle coordinates
		}
}

