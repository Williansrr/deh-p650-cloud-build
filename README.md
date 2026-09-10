# DEH-P650 — Cloud Build V1.0.8.5 DIAG XM1↔XM2

Este repositório compila na nuvem o firmware de diagnóstico do Pioneer DEH-P650.

## Resultado do teste
- XM1 do rádio usa temporariamente o perfil de áudio originalmente usado pelo XM2.
- XM2 do rádio usa temporariamente o BBR6 originalmente usado pelo XM1.
- XM3 permanece HIFI PURE / bypass.

O objetivo é descobrir se o chiado acompanha o BBR6/DSP.

## Como usar
1. Crie um repositório novo no GitHub.
2. Envie TODO o conteúdo desta pasta, inclusive `.github`.
3. Faça o commit.
4. A compilação inicia automaticamente.
5. Abra a aba Actions.
6. Abra `Build DEH-P650 V1.0.8.5 DIAG`.
7. Quando ficar verde, abra a execução.
8. Na área Artifacts, baixe `DEH-P650-V1.0.8.5-DIAG-XM1-XM2`.
9. Extraia o ZIP do artefato.
10. Use `BDK_XM_V1_0_8_5_DIAG_SWAP_XM1_XM2.dehota` pelo OTA normal.

## O que o cloud build usa
O workflow baixa:
https://github.com/WillyBilly06/ESP32-A2DP-SINK-WITH-CODECS-UPDATED

Essa base contém a árvore ESP-IDF 5.5.2 modificada com suporte LDAC.
Depois o workflow aplica o overlay DEH-P650 e compila.

## Importante
Para este diagnóstico use SOMENTE o arquivo `.dehota`.
Os arquivos `bootloader_reference.bin` e `partition-table_reference.bin` são guardados
apenas como referência de build. Não precisam ser gravados.

Se a Action ficar vermelha, abra a etapa que falhou, copie o erro e envie ao ChatGPT.
