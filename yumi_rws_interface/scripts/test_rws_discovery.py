#!/usr/bin/env python3
"""
test_rws_discovery.py — Diagnóstico de endpoints RWS disponíveis no YuMi.

Testa ~30 endpoints RWS organizados por categoria e reporta quais funcionam.
Útil para verificar a conectividade e descobrir as URLs corretas antes de
configurar o cliente RWS.

Uso:
    python3 test_rws_discovery.py [IP]
    python3 test_rws_discovery.py 192.168.125.1

Part of the yumi_rws_interface package.
Project: Collaborative Robotics for RCD Pre-Assembly
Author: Rui Martins (up202108756@edu.fe.up.pt)
Supervisors: Prof. Luís F. Rocha, Prof. [co-orientador]
FEUP/INESCTEC, 2026
"""

import requests
from requests.auth import HTTPDigestAuth
import sys


def test_endpoint(session, base_url, endpoint, auth):
    """Testa um endpoint específico."""
    try:
        url = f"{base_url}{endpoint}"
        r = session.get(url, auth=auth, timeout=3)
        return {
            'url': url,
            'status': r.status_code,
            'ok': r.status_code == 200,
            'content_type': r.headers.get('Content-Type', 'unknown'),
            'content_length': len(r.text),
            'sample': r.text[:300] if r.status_code == 200 else r.text[:150]
        }
    except requests.exceptions.Timeout:
        return {'url': f"{base_url}{endpoint}", 'status': 'timeout', 'ok': False}
    except requests.exceptions.ConnectionError:
        return {'url': f"{base_url}{endpoint}", 'status': 'connection_error', 'ok': False}
    except Exception as e:
        return {'url': f"{base_url}{endpoint}", 'status': f'error: {e}', 'ok': False}


def main():
    # Configuração
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.125.1"
    base_url = f"http://{ip}"
    auth = HTTPDigestAuth("Default User", "robotics")
    
    session = requests.Session()
    session.headers.update({
        'Accept': 'application/xhtml+xml;v=2.0'
    })
    
    print("\n" + "="*70)
    print("DIAGNÓSTICO RWS - ABB YuMi")
    print(f"URL Base: {base_url}")
    print("Credenciais: Default User / robotics")
    print("="*70)
    
    # Lista de endpoints a testar (organizados por categoria)
    endpoints = {
        "Base": [
            "/",
            "/rw",
            "/ctrl",
        ],
        "Sistema": [
            "/rw/system",
            "/rw/system/robotware",
            "/rw/system/options",
            "/ctrl/identity",
        ],
        "Painel": [
            "/rw/panel",
            "/rw/panel/ctrlstate",
            "/rw/panel/opmode",
            "/rw/panel/speedratio",
        ],
        "RAPID": [
            "/rw/rapid",
            "/rw/rapid/tasks",
            "/rw/rapid/execution",
            "/rw/rapid/modules",
        ],
        "Motion System": [
            "/rw/motionsystem",
            "/rw/motionsystem/mechunits",
            "/rw/motionsystem/mechunits/ROB_L",
            "/rw/motionsystem/mechunits/ROB_R",
            "/rw/motionsystem/mechunits/ROB_L/jointtarget",
            "/rw/motionsystem/mechunits/ROB_R/jointtarget",
            "/rw/motionsystem/mechunits/ROB_L/robtarget",
            "/rw/motionsystem/mechunits/ROB_R/robtarget",
        ],
        "I/O": [
            "/rw/iosystem",
            "/rw/iosystem/signals",
            "/rw/iosystem/devices",
        ],
        "Config": [
            "/rw/cfg",
            "/rw/cfg/moc",
        ],
    }
    
    # Testar cada categoria
    working_endpoints = []
    failed_endpoints = []
    
    for category, eps in endpoints.items():
        print(f"\n--- {category} ---")
        for ep in eps:
            result = test_endpoint(session, base_url, ep, auth)
            
            if result['ok']:
                print(f"  ✅ {ep}")
                print(f"     Status: {result['status']}, Content-Type: {result['content_type']}")
                # Mostrar amostra do conteúdo
                sample = result['sample'].replace('\n', ' ').replace('\r', '')[:100]
                print(f"     Sample: {sample}...")
                working_endpoints.append(ep)
            else:
                print(f"  ❌ {ep} - {result['status']}")
                failed_endpoints.append((ep, result['status']))
    
    # Resumo
    print("\n" + "="*70)
    print("RESUMO")
    print("="*70)
    
    print(f"\n✅ Endpoints que funcionam ({len(working_endpoints)}):")
    for ep in working_endpoints:
        print(f"   {ep}")
    
    print(f"\n❌ Endpoints que falharam ({len(failed_endpoints)}):")
    for ep, status in failed_endpoints[:10]:  # Mostrar só os primeiros 10
        print(f"   {ep} - {status}")
    if len(failed_endpoints) > 10:
        print(f"   ... e mais {len(failed_endpoints) - 10}")
    
    # Recomendações
    print("\n" + "="*70)
    print("PRÓXIMOS PASSOS")
    print("="*70)
    
    if "/rw/panel/ctrlstate" in working_endpoints:
        print("✅ Podes ler o estado do controlador")
    else:
        print("⚠️  Endpoint de estado não disponível - verifica autenticação")
    
    if "/rw/motionsystem/mechunits/ROB_L/jointtarget" in working_endpoints:
        print("✅ Podes ler joint positions do braço esquerdo")
    elif "/rw/motionsystem/mechunits/ROB_R/jointtarget" in working_endpoints:
        print("✅ Podes ler joint positions do braço direito")
    else:
        print("⚠️  Endpoints de joints não disponíveis")
        print("   Tenta descobrir os nomes corretos das mechunits:")
        # Verificar se /rw/motionsystem/mechunits funcionou
        if "/rw/motionsystem/mechunits" in working_endpoints:
            print("   Verifica a resposta de /rw/motionsystem/mechunits")
    
    if "/rw/rapid/tasks" in working_endpoints:
        print("✅ Podes ler tasks RAPID")
    
    print("\n" + "="*70)
    print("Copia os endpoints que funcionam para configurar o cliente RWS")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
