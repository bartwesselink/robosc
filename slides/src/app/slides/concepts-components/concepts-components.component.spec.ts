import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ConceptsComponentsComponent } from './concepts-components.component';

describe('ConceptsComponentsComponent', () => {
  let component: ConceptsComponentsComponent;
  let fixture: ComponentFixture<ConceptsComponentsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ ConceptsComponentsComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ConceptsComponentsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
