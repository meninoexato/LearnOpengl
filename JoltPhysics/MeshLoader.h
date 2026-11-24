#pragma once
// Carrega o arquivo .mesh exportado do hier.lnc (bloco 0: suporte mecânico, 17.592 vértices / 19.888 triângulos)
// Formato: uint32 nVerts, uint32 nIndices, float[nVerts*5] (x y z u v), uint32[nIndices]
// Preenche a MeshData do seu main.cpp. 'scale' ajusta o tamanho (a peça original tem ~0,17 m; scale=30 -> ~5 unidades).
#include <cstdio>
#include <cstdint>
#include <vector>
#include <iostream>

static bool LoadMeshFile(const char* path, MeshData& out, float scale = 30.0f)
{
	FILE* f = std::fopen(path, "rb");
	if (!f) { std::cout << "Nao abriu " << path << std::endl; return false; }

	uint32_t nV = 0, nI = 0;
	if (std::fread(&nV, 4, 1, f) != 1 || std::fread(&nI, 4, 1, f) != 1) { std::fclose(f); return false; }

	out.vertices.resize((size_t)nV * 5);
	out.indices.resize(nI);
	bool ok = std::fread(out.vertices.data(), sizeof(float), out.vertices.size(), f) == out.vertices.size()
		&& std::fread(out.indices.data(), sizeof(uint32_t), nI, f) == nI;
	std::fclose(f);
	if (!ok) return false;

	// aplica a escala em x y z (u v ficam como estão), assim OpenGL e Jolt usam a mesma geometria
	for (size_t i = 0; i < out.vertices.size(); i += 5)
	{
		out.vertices[i]     *= scale;
		out.vertices[i + 1] *= scale;
		out.vertices[i + 2] *= scale;
	}
	return true;
}
